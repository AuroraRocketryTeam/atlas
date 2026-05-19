from __future__ import annotations
import socket
import struct
import time
import threading
from dataclasses import dataclass
from typing import Callable, Optional

ESP_IP = "192.168.42.1"
PORT = 5000

MAGIC = 0xA5A55A5A
HEADER_FMT = "!IHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

# payload format
# counter, sim_time, timestamp, ax, ay, az, p, lat, lon, alt
PAYLOAD_FMT = "<IfIfffffff"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FMT)

COMMAND_FMT = "<fBBf"
COMMAND_SIZE = struct.calcsize(COMMAND_FMT)


MSG_TYPE_SIM_INPUT = 1
MSG_TYPE_FC_COMMAND = 2
MSG_TYPE_SIM_RESET = 3

RESET_SIMULATION = "RESET_SIMULATION"

class PeerClosedConnection(ConnectionError):
    """
    Raised only when the peer closes the TCP connection cleanly.

    This is expected when the flight controller performs an FSM transition
    and calls shutdown() + close() on its socket.
    """
    pass

# ---------------------------------------------------------
# Encode message
# ---------------------------------------------------------
def encode_msg(msg_type: int, payload: bytes) -> bytes:
    header = struct.pack(HEADER_FMT, MAGIC, len(payload), msg_type)
    return header + payload


# ---------------------------------------------------------
# Build simulator payload
# ---------------------------------------------------------
def build_payload(seq, t, ax, ay, az, p, lat, lon, alt):
    timestamp = int(time.time())

    return struct.pack(
        PAYLOAD_FMT, 
        seq, t, timestamp, ax, ay, az, p, lat, lon, alt,
    )
def unbuild_payload(payload):
    timestamp = int(time.time())

    seq, t, timestamp, ax, ay, az, p, lat, lon, alt = struct.unpack(
        PAYLOAD_FMT, 
        payload
    )
    return seq, t, timestamp, ax, ay, az, p, lat, lon, alt



# ---------------------------------------------------------
# Decode commands
# ---------------------------------------------------------
def decode_command(payload):
    sim_time, open_main, open_drogue, airbrakes = struct.unpack(COMMAND_FMT, payload)
    return sim_time, bool(open_main), bool(open_drogue), airbrakes
    

# ---------------------------------------------------------
# Receive exactly N bytes
# ---------------------------------------------------------
def recv_all(sock, n):

    data = b""

    while len(data) < n:
        chunk = sock.recv(n - len(data))

        if not chunk:
            raise PeerClosedConnection("peer closed connection")

        data += chunk

    return data


# ---------------------------------------------------------
# Receive one protocol message
# ---------------------------------------------------------
def recv_msg(sock):

    header = recv_all(sock, HEADER_SIZE)

    magic, length, msg_type = struct.unpack(HEADER_FMT, header)

    if magic != MAGIC:
        raise ValueError("Invalid magic")

    payload = recv_all(sock, length) if length else b""

    return msg_type, payload


@dataclass
class CommandHandlers:
    on_open_main: Optional[Callable[[float], None]] = None
    on_open_drogue: Optional[Callable[[float], None]] = None
    on_set_air_brakes: Optional[Callable[[float,float], None]] = None


# TODO: replace with threading.Queue(maxsize=1)
class OneSlotMailbox:
    """
    Single-slot mailbox storing exactly one value.
    Producer: put(value)   -> blocks if slot is full
    Consumer: take()       -> blocks if slot is empty
    """

    def __init__(self):
        self._cv = threading.Condition()
        self._full = False
        self._value = None

    def put(self, value):
        """Blocking put."""
        with self._cv:
            while self._full:
                self._cv.wait()

            self._value = value
            self._full = True

            self._cv.notify_all()

    def take(self):
        """Blocking take."""
        with self._cv:
            while not self._full:
                self._cv.wait()

            v = self._value
            self._value = None
            self._full = False

            self._cv.notify_all()

            return v


# ---------------------------------------------------------
# Main loop
# ---------------------------------------------------------
def tcp_client(esp_connected: threading.Semaphore,
               mailbox: OneSlotMailbox,
               handlers: CommandHandlers):

    announced_connected = False
    stop_requested = False

    # Message flow:
    # sim_data, cmd, sim_data, cmd, ... sim_data, cmd, reset.

    while not stop_requested:
        sock = None

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((ESP_IP, PORT))
            sock.settimeout(3.0)

            if not announced_connected:
                esp_connected.release()
                announced_connected = True
                print("[READY] ESP connection semaphore released")

            while not stop_requested:
                # ===== GET PAYLOAD =====
                payload = mailbox.take()

                if payload == RESET_SIMULATION:
                    frame = encode_msg(MSG_TYPE_SIM_RESET, b"")
                    sock.sendall(frame)
                    print("[RESET] end of simulation, sent reset.")

                    # This is intentional shutdown of the Python TCP client.
                    # Do not reconnect after the final simulation reset.
                    stop_requested = True
                    break

                # ===== ENCODE + SEND =====
                frame = encode_msg(MSG_TYPE_SIM_INPUT, payload)
                sock.sendall(frame)

                # ===== RECEIVE =====
                msg_type, rx_payload = recv_msg(sock)

                # ===== VALIDATE =====
                if msg_type != MSG_TYPE_FC_COMMAND:
                    raise RuntimeError(f"Invalid message type: {msg_type}")

                if len(rx_payload) != COMMAND_SIZE:
                    raise RuntimeError(
                        f"Payload size mismatch: {len(rx_payload)} != {COMMAND_SIZE}"
                    )

                # ===== DECODE =====
                sim_time, open_main, open_drogue, airbrakes = decode_command(rx_payload)

                # ===== HANDLE =====
                if open_main and handlers.on_open_main:
                    handlers.on_open_main(sim_time)

                if open_drogue and handlers.on_open_drogue:
                    handlers.on_open_drogue(sim_time)

                if handlers.on_set_air_brakes:
                    handlers.on_set_air_brakes(sim_time, airbrakes)

        # Expected case:
        # Flight controller FSM transition stopped the task and closed the socket.
        except (
            PeerClosedConnection,
            BrokenPipeError,
            ConnectionResetError,
            ConnectionAbortedError,
        ) as e:
            if not stop_requested:
                print(
                    f"[INFO] FC closed socket during FSM transition: "
                    f"{type(e).__name__}: {e}"
                )

        # Expected during reconnect:
        # The FC closed the old server/task, but the new one is not listening yet.
        except ConnectionRefusedError as e:
            if not stop_requested:
                print(f"[INFO] FC TCP server not ready yet, retrying: {e}")

        # Not treated as normal FSM shutdown:
        # The socket stayed open, but the FC did not answer in time.
        except socket.timeout as e:
            if not stop_requested:
                print(f"[TIMEOUT] FC did not answer in time: {e}")

        # Protocol/data errors:
        # These should not be hidden as normal reconnects.
        except (ValueError, RuntimeError, struct.error) as e:
            print(f"[PROTOCOL ERROR] {type(e).__name__}: {e}")

        # Other OS-level socket errors:
        # Keep visible because these may indicate real network/configuration bugs.
        except OSError as e:
            if not stop_requested:
                print(f"[SOCKET ERROR] errno={e.errno}: {type(e).__name__}: {e}")

        # Actual programming bugs:
        # Do not hide them, otherwise debugging becomes impossible.
        except Exception as e:
            print(f"[BUG] Unexpected error: {type(e).__name__}: {e}")
            raise

        finally:
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass

                try:
                    sock.close()
                except OSError:
                    pass

            if not stop_requested:
                time.sleep(1.0)

    print("[TCP] communication thread stopped")

def tcp_client_thread_start(esp_connected, mailbox, handlers):
    th = threading.Thread(
        target=tcp_client,
        args=(esp_connected, mailbox, handlers),
        daemon=True
    )
    th.start()
    return th