"""TCP framing and lockstep transport for RocketPy HIL.

The producer thread queues one simulator payload into ``OneSlotMailbox``. The
TCP thread takes that payload, sends it as ``MSG_TYPE_SIM_INPUT``, waits for one
``MSG_TYPE_FC_COMMAND`` response, and then updates the command handlers before
acknowledging the producer. This one-packet exchange prevents RocketPy/preflight
code from running ahead of the flight controller.

Recoverable socket errors are handled by closing the socket and reconnecting.
Protocol/data errors are printed separately because they indicate a simulator/FC
contract mismatch rather than a normal reset or reconnect.
"""

from __future__ import annotations

import os
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

ESP_IP = os.getenv("HIL_FC_HOST", "192.168.4.1")
PORT = int(os.getenv("HIL_FC_PORT", "5000"))
DEFAULT_HIL_SAMPLING_RATE_HZ = 50

MAGIC = 0xA5A55A5A
HEADER_FMT = "!IHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

# Payload format:
# sequence_number, hil_sim_time_s, host_unix_time_s,
# ax_m_s2, ay_m_s2, az_m_s2, attitude quaternion w/x/y/z,
# pressure_pa, temperature_k,
# latitude_deg, longitude_deg, altitude_m
PAYLOAD_FMT = "<IfIffffffffffff"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FMT)

# Extended command format:
# command_sim_time_s, open_main, open_drogue, airbrakes_deployment, fsm_state
COMMAND_FMT = "<fBBfB"
COMMAND_SIZE = struct.calcsize(COMMAND_FMT)

# These values intentionally mirror RocketState in IStateMachine.hpp / RocketFSM.
# Do not keep a separate HIL-specific FSM enum.
FSM_STATE_INACTIVE = 0
FSM_STATE_CALIBRATING = 1
FSM_STATE_GROUND_SERVICES = 2
FSM_STATE_READY_FOR_LAUNCH = 3
FSM_STATE_LAUNCH = 4
FSM_STATE_ACCELERATED_FLIGHT = 5
FSM_STATE_BALLISTIC_FLIGHT = 6
FSM_STATE_APOGEE = 7
FSM_STATE_STABILIZATION = 8
FSM_STATE_DECELERATION = 9
FSM_STATE_LANDING = 10
FSM_STATE_RECOVERED = 11

FSM_STATE_NAMES = {
    FSM_STATE_INACTIVE: "INACTIVE",
    FSM_STATE_CALIBRATING: "CALIBRATING",
    FSM_STATE_GROUND_SERVICES: "GROUND_SERVICES",
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


class PeerClosedConnection(ConnectionError):
    """
    Raised only when the peer closes the TCP connection cleanly.

    This is expected when the flight controller performs an FSM transition
    and calls shutdown() + close() on its socket.
    """

    pass


TRANSPORT_ERRORS = (
    PeerClosedConnection,
    BrokenPipeError,
    ConnectionResetError,
    ConnectionAbortedError,
    ConnectionRefusedError,
    socket.timeout,
    OSError,
)


def _transport_error_message(error: BaseException, phase: str) -> str:
    """Return a short reconnect-oriented description for socket failures."""
    if isinstance(error, PeerClosedConnection):
        return f"FC closed TCP connection: {error}"
    if isinstance(error, ConnectionRefusedError):
        return f"FC TCP server not ready at {ESP_IP}:{PORT}: {error}"
    if isinstance(error, socket.timeout):
        return f"FC TCP {phase} timed out for {ESP_IP}:{PORT}: {error}"
    return f"{type(error).__name__}: {error}"


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
    quaternion_w,
    quaternion_x,
    quaternion_y,
    quaternion_z,
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
        float(quaternion_w),
        float(quaternion_x),
        float(quaternion_y),
        float(quaternion_z),
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
        quaternion_w,
        quaternion_x,
        quaternion_y,
        quaternion_z,
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
        quaternion_w,
        quaternion_x,
        quaternion_y,
        quaternion_z,
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

    Producer: put(value) -> blocks until this value is acknowledged.
    Consumer: take()    -> blocks if slot is empty; does not free it.

    This is intentionally one-slot for HIL lockstep. The TCP thread sends one
    simulator sample and waits for the FC command before taking the next value.
    """

    def __init__(self):
        """Create an empty slot shared by the simulator and TCP threads."""
        self._cv = threading.Condition()
        self._pending = None
        self._closed = False

    def put(self, value, timeout_s=30.0):
        """Wait for acknowledgement of this value, or fail on a missing response."""
        deadline = time.monotonic() + timeout_s
        with self._cv:
            # A previous sample still owns the slot until its FC reply is handled.
            while self._pending is not None and not self._closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("HIL mailbox is still full")
                self._cv.wait(remaining)
            if self._closed:
                raise RuntimeError("HIL communication stopped")
            # [payload, completion error, callbacks started]; identity tracks this exchange across reconnects.
            pending = [value, None, False]
            self._pending = pending
            self._cv.notify_all()
            # take() does not release the slot: wait for every command callback to finish.
            while self._pending is pending:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    if pending[2]:
                        # Once callbacks start, wait for their result instead of letting a new sample overtake them.
                        self._cv.wait()
                        continue
                    # No reply entered processing in time; release the slot for the caller to retry.
                    self._pending = None
                    self._cv.notify_all()
                    raise TimeoutError("Timed out waiting for FC response")
                self._cv.wait(remaining)
            # acknowledge() passes protocol or callback errors back to the producer.
            if pending[1] is not None:
                raise pending[1]

    def take(self):
        """Return the exchange to send, or None after stop(); the producer stays blocked."""
        with self._cv:
            while self._pending is None and not self._closed:
                self._cv.wait()
            return self._pending

    def pending(self, exchange):
        """Check whether this exchange still owns the slot after a reconnect."""
        with self._cv:
            return self._pending is exchange

    def stopped(self):
        """Report whether local shutdown was requested."""
        with self._cv:
            return self._closed

    def begin_processing(self, exchange):
        """Claim a validated reply before callbacks; reject an exchange that timed out."""
        with self._cv:
            if self._pending is not exchange:
                return False
            # Keep put() waiting past its deadline while command callbacks run.
            exchange[2] = True
            return True

    def acknowledge(self, exchange, error=None):
        """Release the producer after callbacks, passing any error back to put()."""
        with self._cv:
            if self._pending is exchange:
                exchange[1] = error
                self._pending = None
                self._cv.notify_all()

    def stop(self):
        """Wake both mailbox users and stop the TCP loop without sending a wire message."""
        with self._cv:
            self._closed = True
            if self._pending is not None:
                self._pending[1] = RuntimeError("HIL communication stopped")
                self._pending = None
            self._cv.notify_all()


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
    exchange = None

    # Message flow during a normal run:
    #   startup reset -> reconnect -> calibration samples -> flight samples -> local stop
    #
    # RESET_SIMULATION sends MSG_TYPE_SIM_RESET to the FC, but keeps this Python
    # communication thread alive so it can reconnect and continue with calibration.
    #
    # mailbox.stop() is local-only: it does not send anything to the FC.

    while not stop_requested and not mailbox.stopped():
        sock = None
        socket_phase = "connect"

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(15.0)
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
                exchange = exchange if exchange is not None and mailbox.pending(exchange) else mailbox.take()
                if exchange is None:
                    stop_requested = True
                    break
                payload = exchange[0]

                if payload == RESET_SIMULATION:
                    frame = encode_msg(MSG_TYPE_SIM_RESET, b"")
                    sock.sendall(frame)
                    mailbox.acknowledge(exchange)
                    exchange = None
                    print(
                        "[RESET] Board is rebooting. Reconnect this computer to "
                        "the Aurora SoftAP if it does not reconnect automatically."
                    )

                    # Do not stop the TCP client. The FC may close/recreate its
                    # HIL task while resetting the FSM. Close our current socket
                    # and let the outer loop reconnect.
                    break

                # ===== ENCODE + SEND SIMULATION INPUT =====
                frame = encode_msg(MSG_TYPE_SIM_INPUT, payload)
                socket_phase = "send"
                sock.sendall(frame)

                # ===== RECEIVE FC COMMAND =====
                socket_phase = "receive"
                msg_type, rx_payload = recv_msg(sock)
                socket_phase = "idle"

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

                # The FC reply has no sequence number; its echoed float32 sim_time matches this input sample.
                expected_time_s = struct.unpack_from("<f", payload, 4)[0]
                # Allow float32 rounding, but reject a reply for a different 20 ms sample.
                if not (abs(command_sim_time_s - expected_time_s) <= 0.001):
                    raise RuntimeError(
                        f"FC response time {command_sim_time_s} does not match sample {expected_time_s}"
                    )
                # The producer may have timed out while we received the reply; do not apply stale commands.
                if not mailbox.begin_processing(exchange):
                    exchange = None
                    continue

                # ===== HANDLE =====
                if open_main and handlers.on_open_main:
                    handlers.on_open_main(command_sim_time_s)

                if open_drogue and handlers.on_open_drogue:
                    handlers.on_open_drogue(command_sim_time_s)

                if handlers.on_set_air_brakes:
                    handlers.on_set_air_brakes(command_sim_time_s, airbrakes_deployment)

                if handlers.on_fsm_state:
                    handlers.on_fsm_state(command_sim_time_s, fsm_state)
                # RocketPy may advance only after all actuator and FSM callbacks have completed.
                mailbox.acknowledge(exchange)
                exchange = None

        # Protocol/data errors:
        # These should not be hidden as normal reconnects.
        except (ValueError, RuntimeError, struct.error) as e:
            if exchange is not None:
                mailbox.acknowledge(exchange, e)
                exchange = None
            message = f"{type(e).__name__}: {e}"
            print(f"[PROTOCOL ERROR] {message}")

        # Recoverable transport errors. Startup reset and FC task restarts can
        # close the socket; reconnect and resend the still-pending sample.
        except TRANSPORT_ERRORS as e:
            if not stop_requested:
                message = _transport_error_message(e, socket_phase)
                print(f"[TCP] {message}; reconnecting")

        # Actual programming bugs:
        # Do not hide them, otherwise debugging becomes impossible.
        except Exception as e:
            if exchange is not None:
                mailbox.acknowledge(exchange, e)
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

            if not stop_requested and not mailbox.stopped():
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
