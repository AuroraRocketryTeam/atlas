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
            raise ConnectionError("socket closed")

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

    while True:
        sock = None

        try:
            print(f"[CONNECT] {ESP_IP}:{PORT}")

            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((ESP_IP, PORT))
            sock.settimeout(3.0)

            print(f"[CONNECTED] {ESP_IP}:{PORT}")

            if not announced_connected:
                esp_connected.release()
                announced_connected = True
                print("[RELEASE] esp_connected")

            while True:
                # ===== GET PAYLOAD =====
                try:
                    payload = mailbox.take()
                    print(f"[MAILBOX] got payload len={len(payload)}")
                except Exception as e:
                    print("[MAILBOX ERROR]", e)
                    raise

                # ===== ENCODE =====
                try:
                    frame = encode_msg(MSG_TYPE_SIM_INPUT, payload)
                    print(f"[ENCODE] frame len={len(frame)}")
                except Exception as e:
                    print("[ENCODE ERROR]", e)
                    raise

                # ===== SEND =====
                try:
                    print("[SEND] sending...", end=" ")
                    sock.sendall(frame)
                    print("OK")
                except Exception as e:
                    print("[SEND ERROR]", e)
                    raise

                # ===== RECEIVE =====
                try:
                    print("[RECV] waiting header...", end=" ")
                    msg_type, rx_payload = recv_msg(sock)
                    print(f"OK type={msg_type} len={len(rx_payload)}")
                except Exception as e:
                    print("[RECV ERROR]", e)
                    raise

                # ===== VALIDATE =====
                if msg_type != MSG_TYPE_FC_COMMAND:
                    print(f"[PROTO ERROR] invalid type={msg_type}")
                    raise RuntimeError("Invalid message type")

                if len(rx_payload) != COMMAND_SIZE:
                    print(f"[PROTO ERROR] payload size {len(rx_payload)} != {COMMAND_SIZE}")
                    raise RuntimeError("Payload size mismatch")

                # ===== DECODE =====
                try:
                    sim_time, open_main, open_drogue, airbrakes = decode_command(rx_payload)
                    print(f"[DECODE] t={sim_time:.2f} main={open_main} drogue={open_drogue} air={airbrakes:.2f}")
                except Exception as e:
                    print("[DECODE ERROR]", e)
                    raise

                # ===== HANDLE =====
                try:
                    if open_main and handlers.on_open_main:
                        handlers.on_open_main(sim_time)

                    if open_drogue and handlers.on_open_drogue:
                        handlers.on_open_drogue(sim_time)

                    if handlers.on_set_air_brakes:
                        handlers.on_set_air_brakes(sim_time, airbrakes)

                except Exception as e:
                    print("[HANDLER ERROR]", e)
                    raise

        except (socket.timeout, socket.error, ConnectionError, OSError) as e:
            print(f"[DISCONNECT] {type(e).__name__}: {e}")

        except Exception as e:
            print(f"[ERROR] {type(e).__name__}: {e}")

        finally:
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                    print("[SOCKET] shutdown OK")
                except OSError as e:
                    print("[SOCKET] shutdown failed:", e)

                try:
                    sock.close()
                    print("[SOCKET] close OK")
                except OSError as e:
                    print("[SOCKET] close failed:", e)

            print("[CLOSED] Connection to server.\n")
            time.sleep(1.0)

# def tcp_client(esp_connected : threading.Semaphore, mailbox : OneSlotMailbox, handlers: CommandHandlers):
#     announced_connected = False

#     while True:
#         sock = None
#         try:
#             print(f"[CONNECT] {ESP_IP}:{PORT}")

#             sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             sock.connect((ESP_IP, PORT))

#             print(f"[CONNECTED] {ESP_IP}:{PORT}")

#             if not announced_connected:
#                 esp_connected.release()
#                 announced_connected = True
#                 print(f"[RELEASE] esp_connected")

#             while True:
#                 # print("Mailbox take... ", end="")
#                 payload = mailbox.take()
#                 # print("DONE")

#                 frame = encode_msg(MSG_TYPE_SIM_INPUT, payload)
                
#                 print("send_msg()... ", end="")
#                 sock.sendall(frame)
#                 print("DONE")

#                 print("recv_msg()... ", end="")
#                 msg_type, payload = recv_msg(sock)
#                 print("DONE")


#                 if msg_type != MSG_TYPE_FC_COMMAND:
#                     raise RuntimeError(f"Invalid message type: {msg_type}")
                

#                 if len(payload) != COMMAND_SIZE:
#                     raise RuntimeError(f"FC command payload size mismatch: got {len(payload)}, expected {COMMAND_SIZE}")
                    
#                 sim_time, open_main, open_drogue, airbrakes = decode_command(payload)

#                 if open_main:
#                     handlers.on_open_main(sim_time)

#                 if open_drogue:
#                     handlers.on_open_drogue(sim_time)

#                 handlers.on_set_air_brakes(sim_time, airbrakes)

#                 # print(
#                 #     f"RX COMMAND | sim_time={sim_time:.2f} "
#                 #     f"main={open_main} "
#                 #     f"drogue={open_drogue} "
#                 #     f"airbrakes={airbrakes:.2f}"
#                 # )

#                 # time.sleep(0.5)

#         except (socket.timeout, socket.error, ConnectionError, OSError) as e:
#             print("[DISCONNECT]", e)

#         except Exception as e:
#             print("[ERROR]", e)

#         finally:
#             if sock is not None:
#                 try:
#                     sock.shutdown(socket.SHUT_RDWR)
#                 except OSError:
#                     pass
#                 try:
#                     sock.close()
#                 except OSError:
#                     pass

#             print("[CLOSED] Connection to server.")
#             time.sleep(1.0)        

def tcp_client_thread_start(esp_connected, mailbox, handlers):
    th = threading.Thread(
        target=tcp_client,
        args=(esp_connected, mailbox, handlers),
        daemon=True
    )
    th.start()
    return th