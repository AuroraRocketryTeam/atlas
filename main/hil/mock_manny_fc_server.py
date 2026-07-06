#!/usr/bin/env python3
"""
Self-contained mock Manny / ESP32 flight-controller TCP server for HIL tests.

Run this in one terminal:

    python3 mock_manny_fc_server.py

Run the RocketPy HIL launcher in another terminal:

    HIL_FC_HOST=127.0.0.1 python3 hil_rocketpy.py --rocket fred --sensor-profile clean --calibration-samples=300

This mock intentionally has no argparse. All tunables live in MockConfig below,
so test behavior is visible in one place and can be versioned with the test.
"""

from __future__ import annotations

import math
import socket
import struct
from collections import deque
from dataclasses import dataclass
from statistics import median


# ----------------------------------------------------------------------
# SINGLE CONFIGURATION POINT
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class MockConfig:
    # TCP server used by hil_communication.py.
    bind_host: str = "127.0.0.1"
    bind_port: int = 5000

    # Must match the launcher calibration run. The mock reports GROUND_SERVICES
    # after this many samples, then READY_FOR_LAUNCH after the configured
    # ground-service dwell below.
    calibration_samples_before_ready: int = 300
    ground_services_samples_before_ready: int = 3

    # Launch detection is armed only after READY_FOR_LAUNCH has been reported.
    # Manny detects launch from total accelerometer magnitude.
    launch_accel_threshold_g: float = 3.0
    launch_marker_samples: int = 1

    # Mock powered-flight phase after launch. This is intentionally independent
    # from the airbrakes command: FSM phase progression must stay monotonic.
    accelerated_flight_duration_s: float = 0.8

    # Apogee detection. Median filters and consecutive confirmations prevent a
    # single noisy altitude/pressure sample from declaring apogee.
    apogee_filter_samples: int = 5
    apogee_altitude_drop_m: float = 0.75
    apogee_pressure_rise_pa: float = 8.0
    apogee_confirmation_samples: int = 3
    apogee_marker_samples: int = 2

    # Airbrakes are commanded only during ascent and only inside this AGL band.
    airbrakes_min_agl_m: float = 45.0
    airbrakes_max_agl_m: float = 125.0
    airbrakes_level: float = 0.5

    # Recovery behavior:
    # - "single_main_at_apogee": for configs like Fred with only a Main chute.
    # - "drogue_then_main": drogue at apogee, main below main_deploy_agl_m.
    # Mock FSM flow:
    # CALIBRATING -> GROUND_SERVICES -> READY_FOR_LAUNCH -> LAUNCH -> ACCELERATED_FLIGHT
    # -> BALLISTIC_FLIGHT -> APOGEE -> STABILIZATION -> DECELERATION
    # -> LANDING -> RECOVERED.
    # Thresholds below are AGL. The simulator sends ASL altitude samples; the
    # mock derives pad altitude during calibration and subtracts it.
    recovery_mode: str = "single_main_at_apogee"
    main_deploy_agl_m: float = 80.0
    landing_agl_m: float = 3.0
    landing_marker_samples: int = 2

    # Logging. Set to 0 to disable periodic status lines.
    verbose_every_samples: int = 20

    # Aligns with hil_communication.py: the launcher sends MSG_TYPE_SIM_RESET,
    # then closes its side and reconnects. The mock resets state and closes too.
    close_connection_on_reset: bool = True


CFG = MockConfig()


# ----------------------------------------------------------------------
# HIL WIRE PROTOCOL
# ----------------------------------------------------------------------

MAGIC = 0xA5A55A5A
HEADER_FMT = "!IHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

PAYLOAD_FMT = "<IfIffffffff"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FMT)

COMMAND_FMT = "<fBBfB"
COMMAND_SIZE = struct.calcsize(COMMAND_FMT)

MSG_TYPE_SIM_INPUT = 1
MSG_TYPE_FC_COMMAND = 2
MSG_TYPE_SIM_RESET = 3

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


class PeerClosedConnection(ConnectionError):
    pass


@dataclass(frozen=True)
class SimInput:
    sequence_number: int
    hil_sim_time_s: float
    host_unix_time_s: int
    accel_x_m_s2: float
    accel_y_m_s2: float
    accel_z_m_s2: float
    pressure_pa: float
    temperature_k: float
    latitude_deg: float
    longitude_deg: float
    altitude_m: float

    @property
    def accel_norm_g(self) -> float:
        norm_m_s2 = math.sqrt(
            self.accel_x_m_s2 * self.accel_x_m_s2
            + self.accel_y_m_s2 * self.accel_y_m_s2
            + self.accel_z_m_s2 * self.accel_z_m_s2
        )
        return norm_m_s2 / 9.80665


def fsm_state_name(state: int) -> str:
    return FSM_STATE_NAMES.get(int(state), f"INVALID_ROCKET_STATE({state})")


def recv_exact(sock: socket.socket, length: int) -> bytes:
    data = b""
    while len(data) < length:
        chunk = sock.recv(length - len(data))
        if not chunk:
            raise PeerClosedConnection("peer closed connection")
        data += chunk
    return data


def recv_msg(sock: socket.socket) -> tuple[int, bytes]:
    header = recv_exact(sock, HEADER_SIZE)
    magic, length, msg_type = struct.unpack(HEADER_FMT, header)
    if magic != MAGIC:
        raise ValueError(f"invalid magic: 0x{magic:08x}")
    payload = recv_exact(sock, length) if length else b""
    return msg_type, payload


def encode_msg(msg_type: int, payload: bytes) -> bytes:
    header = struct.pack(HEADER_FMT, MAGIC, len(payload), msg_type)
    return header + payload


def decode_sim_input(payload: bytes) -> SimInput:
    if len(payload) != PAYLOAD_SIZE:
        raise RuntimeError(f"SIM_INPUT size mismatch: {len(payload)} != {PAYLOAD_SIZE}")

    return SimInput(*struct.unpack(PAYLOAD_FMT, payload))


def build_fc_command(
    *,
    command_sim_time_s: float,
    open_main: bool,
    open_drogue: bool,
    airbrakes_deployment: float,
    fsm_state: int,
) -> bytes:
    payload = struct.pack(
        COMMAND_FMT,
        float(command_sim_time_s),
        int(open_main),
        int(open_drogue),
        float(airbrakes_deployment),
        int(fsm_state),
    )
    if len(payload) != COMMAND_SIZE:
        raise RuntimeError(f"FC_COMMAND size mismatch: {len(payload)} != {COMMAND_SIZE}")
    return encode_msg(MSG_TYPE_FC_COMMAND, payload)


# ----------------------------------------------------------------------
# SMART MOCK FSM
# ----------------------------------------------------------------------

class MockMannyState:
    def __init__(self, cfg: MockConfig):
        self.cfg = cfg
        self.reset()

    def reset(self) -> None:
        self.samples_seen = 0
        self.pad_altitude_m: float | None = None
        self.pad_pressure_pa: float | None = None
        self.calibration_altitudes_m: list[float] = []
        self.calibration_pressures_pa: list[float] = []
        self.last_altitude_m: float | None = None
        self.last_pressure_pa: float | None = None

        self.ready_for_launch_reported = False
        self.ground_services_samples_seen = 0
        self.launch_detected = False
        self.launch_sim_time_s: float | None = None
        self.launch_marker_samples_left = 0
        self.apogee_detected = False
        self.apogee_marker_samples_left = 0
        self.apogee_condition_samples = 0
        self.landing_marker_samples_seen = 0

        self.peak_altitude_m = -math.inf
        self.min_pressure_pa = math.inf
        self.filtered_altitudes_m = deque(maxlen=self.cfg.apogee_filter_samples)
        self.filtered_pressures_pa = deque(maxlen=self.cfg.apogee_filter_samples)
        self.last_fsm_state = FSM_STATE_INACTIVE

    def update_and_choose_state(self, sample: SimInput) -> int:
        self.samples_seen += 1

        # 1) Pad calibration. While calibrating, collect ASL altitude and
        # pressure samples; when complete, convert their medians into the pad
        # reference used for all later AGL checks.
        if not self.ready_for_launch_reported:
            self.calibration_altitudes_m.append(sample.altitude_m)
            self.calibration_pressures_pa.append(sample.pressure_pa)

        if self.samples_seen < self.cfg.calibration_samples_before_ready:
            return self._set_state(FSM_STATE_CALIBRATING, sample)

        if not self.ready_for_launch_reported:
            if self.pad_altitude_m is None:
                self._set_pad_reference_from_calibration()

            self.ground_services_samples_seen += 1
            if self.ground_services_samples_seen <= self.cfg.ground_services_samples_before_ready:
                return self._set_state(FSM_STATE_GROUND_SERVICES, sample)

            self.ready_for_launch_reported = True
            return self._set_state(FSM_STATE_READY_FOR_LAUNCH, sample)

        altitude_agl_m = self.altitude_agl_m(sample)

        # 2) Terminal state. Like RocketFSM, the mock stays recovered until the
        # launcher sends SIM_RESET.
        if self.last_fsm_state == FSM_STATE_RECOVERED:
            return self._set_state(FSM_STATE_RECOVERED, sample)

        # 3) Armed on the pad. Wait for acceleration magnitude to cross the
        # launch threshold, then emit LAUNCH for a configurable number of
        # samples so plots/captures can see it.
        if not self.launch_detected:
            return self._pre_launch_state(sample)

        # 4) Powered/ascent flight. After launch, keep tracking the highest
        # filtered altitude and lowest filtered pressure so apogee can be
        # detected by either an altitude drop or a pressure rise.
        filtered_altitude_m, filtered_pressure_pa = self._filtered_apogee_inputs(sample)
        self.peak_altitude_m = max(self.peak_altitude_m, filtered_altitude_m)
        self.min_pressure_pa = min(self.min_pressure_pa, filtered_pressure_pa)

        if self.launch_marker_samples_left > 0:
            self.launch_marker_samples_left -= 1
            return self._set_state(FSM_STATE_LAUNCH, sample)

        if self._in_accelerated_flight(sample):
            return self._set_state(FSM_STATE_ACCELERATED_FLIGHT, sample)

        # 5) Ballistic flight until apogee is confirmed. The confirmation
        # counter prevents one noisy sample from jumping into recovery.
        if not self.apogee_detected:
            if self._apogee_condition(filtered_altitude_m, filtered_pressure_pa):
                self.apogee_condition_samples += 1
            else:
                self.apogee_condition_samples = 0

            if self.apogee_condition_samples >= self.cfg.apogee_confirmation_samples:
                self.apogee_detected = True
                self.apogee_marker_samples_left = max(1, self.cfg.apogee_marker_samples)

        if self.apogee_marker_samples_left > 0:
            self.apogee_marker_samples_left -= 1
            return self._set_state(FSM_STATE_APOGEE, sample)

        # 6) Recovery. This helper mirrors the RocketFSM recovery order without
        # trying to emulate exact embedded timing:
        # STABILIZATION -> DECELERATION -> LANDING -> RECOVERED.
        if self.apogee_detected:
            return self._recovery_state(sample, altitude_agl_m)

        return self._set_state(FSM_STATE_BALLISTIC_FLIGHT, sample)

    def _pre_launch_state(self, sample: SimInput) -> int:
        if not self._launch_condition(sample):
            return self._set_state(FSM_STATE_READY_FOR_LAUNCH, sample)

        self.launch_detected = True
        self.launch_sim_time_s = sample.hil_sim_time_s
        self.launch_marker_samples_left = max(1, self.cfg.launch_marker_samples) - 1
        self.peak_altitude_m = sample.altitude_m
        self.min_pressure_pa = sample.pressure_pa
        self.filtered_altitudes_m.clear()
        self.filtered_pressures_pa.clear()
        return self._set_state(FSM_STATE_LAUNCH, sample)

    def _recovery_state(self, sample: SimInput, altitude_agl_m: float) -> int:
        # LANDING and RECOVERED are intentionally close in the mock. LANDING is
        # held briefly so it appears in captures, then RECOVERED is terminal.
        if self.last_fsm_state == FSM_STATE_LANDING:
            self.landing_marker_samples_seen += 1
            if self.landing_marker_samples_seen >= self.cfg.landing_marker_samples:
                return self._set_state(FSM_STATE_RECOVERED, sample)
            return self._set_state(FSM_STATE_LANDING, sample)

        # STABILIZATION stands in for the drogue/stabilization phase. Once AGL
        # reaches main_deploy_agl_m, the mock moves to DECELERATION.
        if self.last_fsm_state == FSM_STATE_STABILIZATION:
            if altitude_agl_m <= self.cfg.main_deploy_agl_m:
                return self._set_state(FSM_STATE_DECELERATION, sample)
            return self._set_state(FSM_STATE_STABILIZATION, sample)

        # First recovery sample after APOGEE enters STABILIZATION.
        if self.last_fsm_state != FSM_STATE_DECELERATION:
            return self._set_state(FSM_STATE_STABILIZATION, sample)

        # DECELERATION continues until near-pad AGL. Touchdown starts LANDING.
        if altitude_agl_m <= self.cfg.landing_agl_m:
            self.landing_marker_samples_seen = 1
            return self._set_state(FSM_STATE_LANDING, sample)

        return self._set_state(FSM_STATE_DECELERATION, sample)

    def build_outputs(self, sample: SimInput, fsm_state: int) -> tuple[bool, bool, float]:
        airbrakes = self.airbrakes_deployment(sample)

        if self.cfg.recovery_mode == "single_main_at_apogee":
            open_drogue = False
            open_main = self.apogee_detected
        elif self.cfg.recovery_mode == "drogue_then_main":
            open_drogue = self.apogee_detected
            open_main = (
                self.apogee_detected
                and self.altitude_agl_m(sample) <= self.cfg.main_deploy_agl_m
            )
        else:
            raise ValueError(f"unknown recovery_mode: {self.cfg.recovery_mode!r}")

        if fsm_state in (FSM_STATE_LANDING, FSM_STATE_RECOVERED):
            airbrakes = 0.0

        return open_main, open_drogue, airbrakes

    def altitude_agl_m(self, sample: SimInput) -> float:
        if self.pad_altitude_m is None:
            return 0.0
        return sample.altitude_m - self.pad_altitude_m

    def _set_pad_reference_from_calibration(self) -> None:
        if not self.calibration_altitudes_m or not self.calibration_pressures_pa:
            raise RuntimeError("cannot compute pad reference without calibration samples")

        self.pad_altitude_m = float(median(self.calibration_altitudes_m))
        self.pad_pressure_pa = float(median(self.calibration_pressures_pa))

    def airbrakes_deployment(self, sample: SimInput) -> float:
        if not self.launch_detected or self.apogee_detected:
            return 0.0

        if not self._is_ascending(sample):
            return 0.0

        altitude_agl_m = self.altitude_agl_m(sample)
        if self.cfg.airbrakes_min_agl_m <= altitude_agl_m <= self.cfg.airbrakes_max_agl_m:
            return self.cfg.airbrakes_level

        return 0.0

    def _launch_condition(self, sample: SimInput) -> bool:
        return sample.accel_norm_g >= self.cfg.launch_accel_threshold_g

    def _filtered_apogee_inputs(self, sample: SimInput) -> tuple[float, float]:
        self.filtered_altitudes_m.append(sample.altitude_m)
        self.filtered_pressures_pa.append(sample.pressure_pa)
        return (
            float(median(self.filtered_altitudes_m)),
            float(median(self.filtered_pressures_pa)),
        )

    def _apogee_condition(self, altitude_m: float, pressure_pa: float) -> bool:
        altitude_drop_m = self.peak_altitude_m - altitude_m
        altitude_says_apogee = altitude_drop_m >= self.cfg.apogee_altitude_drop_m

        pressure_rise_pa = pressure_pa - self.min_pressure_pa
        pressure_says_apogee = pressure_rise_pa >= self.cfg.apogee_pressure_rise_pa

        return altitude_says_apogee or pressure_says_apogee

    def _in_accelerated_flight(self, sample: SimInput) -> bool:
        if self.launch_sim_time_s is None:
            return False

        time_since_launch_s = sample.hil_sim_time_s - self.launch_sim_time_s
        return time_since_launch_s <= self.cfg.accelerated_flight_duration_s

    def _is_ascending(self, sample: SimInput) -> bool:
        if self.last_altitude_m is None:
            altitude_increasing = True
        else:
            altitude_increasing = sample.altitude_m >= self.last_altitude_m

        if self.last_pressure_pa is None:
            pressure_decreasing = True
        else:
            pressure_decreasing = sample.pressure_pa <= self.last_pressure_pa

        return altitude_increasing or pressure_decreasing

    def _set_state(self, state: int, sample: SimInput) -> int:
        self.last_fsm_state = state
        self.last_altitude_m = sample.altitude_m
        self.last_pressure_pa = sample.pressure_pa
        return state


# ----------------------------------------------------------------------
# SERVER
# ----------------------------------------------------------------------

def handle_client(conn: socket.socket, addr, fsm: MockMannyState, cfg: MockConfig) -> None:
    print(f"[MOCK MANNY] client connected: {addr}")

    with conn:
        while True:
            try:
                msg_type, payload = recv_msg(conn)

                if msg_type == MSG_TYPE_SIM_RESET:
                    print("[MOCK MANNY] received SIM_RESET: resetting state")
                    fsm.reset()
                    if cfg.close_connection_on_reset:
                        print("[MOCK MANNY] closing connection to emulate FC restart")
                        return
                    continue

                if msg_type != MSG_TYPE_SIM_INPUT:
                    print(f"[MOCK MANNY] unexpected message type: {msg_type}")
                    return

                sample = decode_sim_input(payload)
                fsm_state = fsm.update_and_choose_state(sample)
                open_main, open_drogue, airbrakes = fsm.build_outputs(sample, fsm_state)

                frame = build_fc_command(
                    command_sim_time_s=sample.hil_sim_time_s,
                    open_main=open_main,
                    open_drogue=open_drogue,
                    airbrakes_deployment=airbrakes,
                    fsm_state=fsm_state,
                )
                conn.sendall(frame)

                if (
                    cfg.verbose_every_samples > 0
                    and sample.sequence_number % cfg.verbose_every_samples == 0
                ):
                    print_status_line(sample, fsm, fsm_state, open_main, open_drogue, airbrakes)

            except PeerClosedConnection:
                print("[MOCK MANNY] client closed connection")
                return
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError) as exc:
                print(f"[MOCK MANNY] connection closed by peer: {type(exc).__name__}: {exc}")
                return
            except (RuntimeError, ValueError, struct.error) as exc:
                print(f"[MOCK MANNY] protocol error: {type(exc).__name__}: {exc}")
                return


def print_status_line(
    sample: SimInput,
    fsm: MockMannyState,
    fsm_state: int,
    open_main: bool,
    open_drogue: bool,
    airbrakes: float,
) -> None:
    print(
        "[MOCK MANNY] "
        f"seq={sample.sequence_number} "
        f"read={fsm.samples_seen} "
        f"sim_time={sample.hil_sim_time_s:.3f}s "
        f"agl={fsm.altitude_agl_m(sample):.2f}m "
        f"alt={sample.altitude_m:.2f}m "
        f"p={sample.pressure_pa:.2f}Pa "
        f"|a|={sample.accel_norm_g:.2f}g "
        f"peak={fsm.peak_altitude_m:.2f}m "
        f"fsm={fsm_state_name(fsm_state)} "
        f"main={int(open_main)} "
        f"drogue={int(open_drogue)} "
        f"airbrakes={airbrakes:.2f}"
    )


def serve(cfg: MockConfig) -> None:
    fsm = MockMannyState(cfg)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((cfg.bind_host, cfg.bind_port))
    server.listen(1)

    print(f"[MOCK MANNY] listening on {cfg.bind_host}:{cfg.bind_port}")
    print("[MOCK MANNY] Ctrl+C to stop")
    print("[MOCK MANNY] config:")
    print(f"  calibration_samples_before_ready = {cfg.calibration_samples_before_ready}")
    print(f"  launch_accel_threshold_g         = {cfg.launch_accel_threshold_g}")
    print(f"  accelerated_flight_duration_s    = {cfg.accelerated_flight_duration_s}")
    print(f"  apogee_filter_samples            = {cfg.apogee_filter_samples}")
    print(f"  apogee_altitude_drop_m           = {cfg.apogee_altitude_drop_m}")
    print(f"  apogee_confirmation_samples      = {cfg.apogee_confirmation_samples}")
    print(f"  airbrakes_agl_band_m             = {cfg.airbrakes_min_agl_m}..{cfg.airbrakes_max_agl_m}")
    print(f"  airbrakes_level                  = {cfg.airbrakes_level}")
    print(f"  recovery_mode                    = {cfg.recovery_mode}")
    print(f"  main_deploy_agl_m                = {cfg.main_deploy_agl_m}")
    print(f"  landing_agl_m                    = {cfg.landing_agl_m}")
    print(f"  landing_marker_samples           = {cfg.landing_marker_samples}")
    print("")

    try:
        while True:
            conn, addr = server.accept()
            handle_client(conn, addr, fsm, cfg)
    except KeyboardInterrupt:
        print("\n[MOCK MANNY] stopped by user")
    finally:
        server.close()


if __name__ == "__main__":
    serve(CFG)
