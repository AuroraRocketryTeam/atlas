from __future__ import annotations

import os
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

ESP_IP = os.getenv("HIL_FC_HOST", "192.168.42.1")
PORT = int(os.getenv("HIL_FC_PORT", "5000"))

MAGIC = 0xA5A55A5A
HEADER_FMT = "!IHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

# Payload format:
# sequence_number, hil_sim_time_s, host_unix_time_s,
# ax_m_s2, ay_m_s2, az_m_s2, pressure_pa, temperature_k,
# latitude_deg, longitude_deg, altitude_m
PAYLOAD_FMT = "<IfIffffffff"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FMT)

# Extended command format:
# command_sim_time_s, open_main, open_drogue, airbrakes_deployment, fsm_state
COMMAND_FMT = "<fBBfB"
COMMAND_SIZE = struct.calcsize(COMMAND_FMT)

# These values intentionally mirror RocketState in IStateMachine.hpp / RocketFSM.
# Do not keep a separate HIL-specific FSM enum.
FSM_STATE_INACTIVE = 0
FSM_STATE_CALIBRATING = 1
FSM_STATE_READY_FOR_LAUNCH = 2
FSM_STATE_LAUNCH = 3
FSM_STATE_ACCELERATED_FLIGHT = 4
FSM_STATE_BALLISTIC_FLIGHT = 5
FSM_STATE_APOGEE = 6
FSM_STATE_STABILIZATION = 7
FSM_STATE_DECELERATION = 8
FSM_STATE_LANDING = 9
FSM_STATE_RECOVERED = 10

FSM_STATE_NAMES = {
    FSM_STATE_INACTIVE: "INACTIVE",
    FSM_STATE_CALIBRATING: "CALIBRATING",
    FSM_STATE_READY_FOR_LAUNCH: "READY_FOR_LAUNCH",
    FSM_STATE_LAUNCH: "LAUNCH",
    FSM_STATE_ACCELERATED_FLIGHT: "ACCELERATED_FLIGHT",
    FSM_STATE_BALLISTIC_FLIGHT: "BALLISTIC_FLIGHT",
    FSM_STATE_APOGEE: "APOGEE",
    FSM_STATE_STABILIZATION: "STABILIZATION",
    FSM_STATE_DECELERATION: "DECELERATION",
    FSM_STATE_LANDING: "LANDING",
    FSM_STATE_RECOVERED: "RECOVERED",
}

MSG_TYPE_SIM_INPUT = 1
MSG_TYPE_FC_COMMAND = 2
MSG_TYPE_SIM_RESET = 3

# Control messages used only inside the Python process.
# They are not sent as payloads.
RESET_SIMULATION = "RESET_SIMULATION"
STOP_COMMUNICATION = "STOP_COMMUNICATION"


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
def build_sim_input_payload(
    sequence_number,
    hil_sim_time_s,
    ax_m_s2,
    ay_m_s2,
    az_m_s2,
    pressure_pa,
    temperature_k,
    latitude_deg,
    longitude_deg,
    altitude_m,
):
    host_unix_time_s = int(time.time())

    return struct.pack(
        PAYLOAD_FMT,
        sequence_number,
        float(hil_sim_time_s),
        host_unix_time_s,
        float(ax_m_s2),
        float(ay_m_s2),
        float(az_m_s2),
        float(pressure_pa),
        float(temperature_k),
        float(latitude_deg),
        float(longitude_deg),
        float(altitude_m),
    )


def decode_sim_input_payload(payload):
    (
        sequence_number,
        hil_sim_time_s,
        host_unix_time_s,
        ax_m_s2,
        ay_m_s2,
        az_m_s2,
        pressure_pa,
        temperature_k,
        latitude_deg,
        longitude_deg,
        altitude_m,
    ) = struct.unpack(
        PAYLOAD_FMT,
        payload,
    )
    return (
        sequence_number,
        hil_sim_time_s,
        host_unix_time_s,
        ax_m_s2,
        ay_m_s2,
        az_m_s2,
        pressure_pa,
        temperature_k,
        latitude_deg,
        longitude_deg,
        altitude_m,
    )


# ---------------------------------------------------------
# Decode commands
# ---------------------------------------------------------
def decode_fc_command(payload):
    if len(payload) != COMMAND_SIZE:
        raise RuntimeError(f"Payload size mismatch: {len(payload)} != {COMMAND_SIZE}")

    command_sim_time_s, open_main, open_drogue, airbrakes_deployment, fsm_state = struct.unpack(
        COMMAND_FMT,
        payload,
    )
    return (
        command_sim_time_s,
        bool(open_main),
        bool(open_drogue),
        airbrakes_deployment,
        int(fsm_state),
    )


def fsm_state_name(state):
    return FSM_STATE_NAMES.get(int(state), f"INVALID_ROCKET_STATE({state})")


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
    on_set_air_brakes: Optional[Callable[[float, float], None]] = None
    on_fsm_state: Optional[Callable[[float, int], None]] = None


class OneSlotMailbox:
    """
    Single-slot mailbox storing exactly one value.

    Producer: put(value) -> blocks if slot is full.
    Consumer: take()    -> blocks if slot is empty.

    This is intentionally one-slot for HIL lockstep. The TCP thread sends one
    simulator sample and waits for the FC command before taking the next value.
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

            value = self._value
            self._value = None
            self._full = False

            self._cv.notify_all()

            return value


# ---------------------------------------------------------
# Main loop
# ---------------------------------------------------------
def tcp_client(
    esp_connected: threading.Semaphore,
    mailbox: OneSlotMailbox,
    handlers: CommandHandlers,
    esp_reconnected: Optional[threading.Semaphore] = None,
):
    announced_first_connection = False
    stop_requested = False

    # Message flow during a normal run:
    #   startup reset -> reconnect -> calibration samples -> flight samples -> local stop
    #
    # RESET_SIMULATION sends MSG_TYPE_SIM_RESET to the FC, but keeps this Python
    # communication thread alive so it can reconnect and continue with calibration.
    #
    # STOP_COMMUNICATION is local-only: it does not send anything to the FC. It
    # simply closes the current socket and exits the thread cleanly.

    while not stop_requested:
        sock = None

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            sock.connect((ESP_IP, PORT))

            if not announced_first_connection:
                esp_connected.release()
                announced_first_connection = True
                print("[READY] ESP connection semaphore released")
            else:
                if esp_reconnected is not None:
                    esp_reconnected.release()
                print("[READY] ESP reconnected")

            while not stop_requested:
                # ===== GET PAYLOAD / CONTROL MESSAGE =====
                payload = mailbox.take()

                if payload == RESET_SIMULATION:
                    frame = encode_msg(MSG_TYPE_SIM_RESET, b"")
                    sock.sendall(frame)
                    print("[RESET] startup reset sent")

                    # Do not stop the TCP client. The FC may close/recreate its
                    # HIL task while resetting the FSM. Close our current socket
                    # and let the outer loop reconnect.
                    break

                if payload == STOP_COMMUNICATION:
                    print("[TCP] stop requested, closing socket without FC reset")
                    stop_requested = True
                    break

                # ===== ENCODE + SEND SIMULATION INPUT =====
                frame = encode_msg(MSG_TYPE_SIM_INPUT, payload)
                sock.sendall(frame)

                # ===== RECEIVE FC COMMAND =====
                msg_type, rx_payload = recv_msg(sock)

                # ===== VALIDATE =====
                if msg_type != MSG_TYPE_FC_COMMAND:
                    raise RuntimeError(f"Invalid message type: {msg_type}")

                if len(rx_payload) != COMMAND_SIZE:
                    raise RuntimeError(
                        f"Payload size mismatch: {len(rx_payload)} != {COMMAND_SIZE}"
                    )

                # ===== DECODE =====
                (
                    command_sim_time_s,
                    open_main,
                    open_drogue,
                    airbrakes_deployment,
                    fsm_state,
                ) = decode_fc_command(rx_payload)

                # ===== HANDLE =====
                if open_main and handlers.on_open_main:
                    handlers.on_open_main(command_sim_time_s)

                if open_drogue and handlers.on_open_drogue:
                    handlers.on_open_drogue(command_sim_time_s)

                if handlers.on_set_air_brakes:
                    handlers.on_set_air_brakes(command_sim_time_s, airbrakes_deployment)

                if handlers.on_fsm_state:
                    handlers.on_fsm_state(command_sim_time_s, fsm_state)

        # Expected during startup reset, and still tolerated as a recoverable transport event.
        except (
            PeerClosedConnection,
            BrokenPipeError,
            ConnectionResetError,
            ConnectionAbortedError,
        ) as e:
            if not stop_requested:
                print(
                    f"[INFO] FC closed TCP connection, reconnecting: "
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


def tcp_client_thread_start(
    esp_connected,
    mailbox,
    handlers,
    esp_reconnected=None,
):
    th = threading.Thread(
        target=tcp_client,
        args=(esp_connected, mailbox, handlers, esp_reconnected),
        daemon=True,
    )
    th.start()
    return th
