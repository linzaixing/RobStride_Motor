#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""EDULITE-A3 private-protocol CAN motors bus for LeRobot (RobStride).

This is a drop-in alternative to :class:`lerobot.motors.robstride.RobstrideMotorsBus`
that speaks RobStride's **private protocol** over **CAN 2.0 extended (29-bit) frames**,
byte-for-byte compatible with the upstream EDULITE_A3 SDK
(``el_a3_sdk/el_a3_sdk/can_driver.py``). Use this when the arm's motors are left in
their default private-protocol mode (so the EDULITE SDK/ROS stack keeps working),
instead of switching them to MIT mode for lerobot's built-in robstride bus.

29-bit extended ID layout (private protocol):
    bit 28..24 : comm type  (1 = motion control, 2 = feedback, 3 = enable, ...)
    bit 23..8  : data area 2 (host id, or feed-forward torque for motion control)
    bit  7..0  : target motor CAN id

Motion-control (type 1) payload: big-endian ``>HHHH`` = position, velocity, kp, kd
(each linearly mapped to uint16); feed-forward torque is carried in the ID.
Feedback (type 2) payload: big-endian ``>HHHH`` = position, velocity, torque, temp*10.

All public position/velocity values on this bus are in **degrees / deg·s⁻¹** to match
the lerobot ``MotorsBus`` convention; conversion to the motors' native radians happens
internally.

NOTE: This module has been validated at the byte-encoding level (see
``tests/motors/test_edulite_robstride_bus.py``) but not yet on physical hardware.
"""

import logging
import socket
import struct
import threading
import time

import numpy as np

from lerobot.motors.motors_bus import Motor, MotorCalibration, MotorsBusBase, Value
from lerobot.utils.errors import DeviceNotConnectedError

logger = logging.getLogger(__name__)

# ---- SocketCAN / frame constants (mirror el_a3_sdk/can_driver.py) --------------
CAN_FRAME_FMT = "=IB3x8s"  # can_id(4) + dlc(1) + pad(3) + data(8) = 16 bytes
CAN_FRAME_SIZE = 16
CAN_EFF_FLAG = 0x80000000  # extended-frame flag
CAN_EFF_MASK = 0x1FFFFFFF  # 29-bit id mask
SOL_CAN_RAW = 101
CAN_RAW_RECV_OWN_MSGS = 4

# ---- private-protocol comm types ----------------------------------------------
COMM_MOTION_CONTROL = 1
COMM_FEEDBACK = 2
COMM_ENABLE = 3
COMM_DISABLE = 4
COMM_SET_ZERO = 6

DEFAULT_HOST_CAN_ID = 0xFD

# ---- per-model parameter ranges (rad, rad/s, N·m) ; mirror MOTOR_PARAMS --------
# kp range 0..500, kd range 0..5, position ±12.57 rad for every model.
_P_MIN, _P_MAX = -12.57, 12.57
_KP_MIN, _KP_MAX = 0.0, 500.0
_KD_MIN, _KD_MAX = 0.0, 5.0
_MODEL_RANGES: dict[str, dict[str, float]] = {
    # RobStride RS00 (lerobot model "O0")
    "O0": {"v_min": -33.0, "v_max": 33.0, "t_min": -14.0, "t_max": 14.0},
    # RobStride RS05 (lerobot model "O5")
    "O5": {"v_min": -50.0, "v_max": 50.0, "t_min": -5.5, "t_max": 5.5},
    # EL05 (lerobot model "ELO5")
    "ELO5": {"v_min": -50.0, "v_max": 50.0, "t_min": -6.0, "t_max": 6.0},
}


# ---- pure encode/decode helpers (unit-testable, no socket) ---------------------
def float_to_uint16(x: float, x_min: float, x_max: float) -> int:
    """Linear map of a float onto uint16 [0, 65535] (matches el_a3_sdk.utils)."""
    x = max(x_min, min(x_max, x))
    return int((x - x_min) * 65535.0 / (x_max - x_min))


def uint16_to_float(x_int: int, x_min: float, x_max: float) -> float:
    """Inverse of :func:`float_to_uint16`."""
    return x_int * (x_max - x_min) / 65535.0 + x_min


def build_extended_can_id(comm_type: int, data_area2: int, target_id: int) -> int:
    """Build a 29-bit private-protocol arbitration id + EFF flag."""
    can_id = ((comm_type & 0x1F) << 24) | ((data_area2 & 0xFFFF) << 8) | (target_id & 0xFF)
    return can_id | CAN_EFF_FLAG


def encode_motion_control(
    position_deg: float,
    velocity_deg: float,
    kp: float,
    kd: float,
    torque: float,
    ranges: dict[str, float],
) -> tuple[int, bytes]:
    """Encode a type-1 motion-control command.

    Returns ``(data_area2, data8)`` where ``data_area2`` is the feed-forward-torque
    uint16 that must go into the arbitration id, and ``data8`` is the 8-byte payload.
    """
    pos_raw = float_to_uint16(np.radians(position_deg), _P_MIN, _P_MAX)
    vel_raw = float_to_uint16(np.radians(velocity_deg), ranges["v_min"], ranges["v_max"])
    kp_raw = float_to_uint16(kp, _KP_MIN, _KP_MAX)
    kd_raw = float_to_uint16(kd, _KD_MIN, _KD_MAX)
    tau_raw = float_to_uint16(torque, ranges["t_min"], ranges["t_max"])
    data = struct.pack(">HHHH", pos_raw, vel_raw, kp_raw, kd_raw)
    return tau_raw, data


def decode_feedback(data: bytes, ranges: dict[str, float]) -> dict[str, float]:
    """Decode a type-2 feedback payload into physical units (degrees)."""
    pos_raw, vel_raw, tau_raw, temp_raw = struct.unpack(">HHHH", data)
    return {
        "position": np.degrees(uint16_to_float(pos_raw, _P_MIN, _P_MAX)),
        "velocity": np.degrees(uint16_to_float(vel_raw, ranges["v_min"], ranges["v_max"])),
        "torque": uint16_to_float(tau_raw, ranges["t_min"], ranges["t_max"]),
        "temperature": temp_raw / 10.0,
    }


class EduliteRobstrideBus(MotorsBusBase):
    """RobStride private-protocol (extended-frame) CAN bus, EDULITE-compatible."""

    def __init__(
        self,
        port: str,
        motors: dict[str, Motor],
        calibration: dict[str, MotorCalibration] | None = None,
        host_can_id: int = DEFAULT_HOST_CAN_ID,
        # accepted for signature-compatibility with RobstrideMotorsBus (ignored here):
        can_interface: str = "socketcan",
        use_can_fd: bool = False,
        bitrate: int = 1_000_000,
    ):
        super().__init__(port, motors, calibration)
        self.host_can_id = host_can_id
        self._socket: socket.socket | None = None

        # per-motor state
        self._ranges: dict[str, dict[str, float]] = {}
        self._gains: dict[str, dict[str, float]] = {}
        self._id_to_name: dict[int, str] = {}
        self._state: dict[str, dict[str, float]] = {}
        for name, motor in self.motors.items():
            model = (motor.model or "O0").upper()
            if model not in _MODEL_RANGES:
                raise ValueError(f"Unsupported EDULITE motor model '{model}' for '{name}'")
            self._ranges[name] = _MODEL_RANGES[model]
            self._gains[name] = {"kp": 10.0, "kd": 0.5}
            self._id_to_name[motor.id] = name
            self._state[name] = {"position": 0.0, "velocity": 0.0, "torque": 0.0, "temperature": 0.0}

        # background receive thread
        self._recv_running = False
        self._recv_thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()

    # ------------------------------------------------------------------ connection
    @property
    def is_connected(self) -> bool:
        return self._socket is not None

    def connect(self, handshake: bool = True) -> None:
        if self.is_connected:
            raise DeviceNotConnectedError(f"{self.__class__.__name__}('{self.port}') already connected.")
        try:
            self._socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self._socket.bind((self.port,))
            self._socket.setsockopt(SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, struct.pack("i", 0))
            self._socket.settimeout(0.1)
        except Exception as e:
            self._socket = None
            raise ConnectionError(f"Failed to open CAN interface '{self.port}': {e}") from e
        self._start_receive_thread()
        logger.info(f"{self.__class__.__name__} connected on {self.port} (private protocol).")

    def disconnect(self, disable_torque: bool = True) -> None:
        if disable_torque:
            try:
                self.disable_torque()
            except Exception as e:  # noqa: BLE001
                logger.warning(f"Failed to disable torque on disconnect: {e}")
        self._stop_receive_thread()
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        logger.info(f"{self.__class__.__name__} disconnected.")

    # ------------------------------------------------------------------ tx/rx
    def _sock(self) -> socket.socket:
        if self._socket is None:
            raise DeviceNotConnectedError(f"{self.__class__.__name__}('{self.port}') is not connected.")
        return self._socket

    def _send(self, can_id: int, data: bytes) -> None:
        frame = struct.pack(CAN_FRAME_FMT, can_id, 8, data.ljust(8, b"\x00"))
        with self._send_lock:
            self._sock().send(frame)

    def _start_receive_thread(self) -> None:
        if self._recv_running:
            return
        self._recv_running = True
        self._recv_thread = threading.Thread(
            target=self._receive_loop, daemon=True, name=f"edulite_can_recv_{self.port}"
        )
        self._recv_thread.start()

    def _stop_receive_thread(self) -> None:
        self._recv_running = False
        if self._recv_thread is not None:
            self._recv_thread.join(timeout=1.0)
            self._recv_thread = None

    def _receive_loop(self) -> None:
        while self._recv_running:
            try:
                frame = self._socket.recv(CAN_FRAME_SIZE)  # type: ignore[union-attr]
            except (TimeoutError, OSError):
                continue
            if len(frame) == CAN_FRAME_SIZE:
                self._parse_frame(frame)

    def _parse_frame(self, raw: bytes) -> None:
        can_id_raw, _dlc = struct.unpack("=IB", raw[:5])
        data = raw[8:16]
        if not (can_id_raw & CAN_EFF_FLAG):
            return
        can_id = can_id_raw & CAN_EFF_MASK
        comm_type = (can_id >> 24) & 0x1F
        motor_id = (can_id >> 8) & 0xFF
        if comm_type == COMM_FEEDBACK and motor_id in self._id_to_name:
            name = self._id_to_name[motor_id]
            decoded = decode_feedback(data, self._ranges[name])
            with self._lock:
                self._state[name] = decoded

    # ------------------------------------------------------------------ commands
    def _poll(self, name: str) -> None:
        """Send a zero-gain, zero-torque motion-control frame to elicit feedback."""
        motor_id = self.motors[name].id
        cached_pos = self._state[name]["position"]
        tau_raw, data = encode_motion_control(cached_pos, 0.0, 0.0, 0.0, 0.0, self._ranges[name])
        self._send(build_extended_can_id(COMM_MOTION_CONTROL, tau_raw, motor_id), data)

    def _enable(self, name: str) -> None:
        motor_id = self.motors[name].id
        self._send(build_extended_can_id(COMM_ENABLE, self.host_can_id, motor_id), bytes(8))

    def _disable(self, name: str) -> None:
        motor_id = self.motors[name].id
        self._send(build_extended_can_id(COMM_DISABLE, self.host_can_id, motor_id), bytes(8))

    def configure_motors(self) -> None:
        for name in self.motors:
            self._enable(name)
            time.sleep(0.005)

    def enable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        for name in self._names(motors):
            self._enable(name)

    def disable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        for name in self._names(motors):
            self._disable(name)

    def set_zero_position(self, motors: str | list[str] | None = None) -> None:
        for name in self._names(motors):
            motor_id = self.motors[name].id
            data = bytes([1]) + bytes(7)
            self._send(build_extended_can_id(COMM_SET_ZERO, self.host_can_id, motor_id), data)
            time.sleep(0.01)

    # ------------------------------------------------------------------ read/write
    def read(self, data_name: str, motor: str) -> Value:
        self._poll(motor)
        time.sleep(0.002)
        return self._cached(data_name, motor)

    def write(self, data_name: str, motor: str, value: Value) -> None:
        if data_name in ("Kp", "Kd"):
            self._gains[motor][data_name.lower()] = float(value)
        elif data_name == "Goal_Position":
            self._send_goal(motor, float(value))
        else:
            raise ValueError(f"Writing '{data_name}' not supported by {self.__class__.__name__}")

    def sync_read(self, data_name: str, motors: str | list[str] | None = None) -> dict[str, Value]:
        names = self._names(motors)
        for name in names:
            self._poll(name)
        time.sleep(0.003)
        return {name: self._cached(data_name, name) for name in names}

    def sync_write(self, data_name: str, values: dict[str, Value]) -> None:
        if data_name in ("Kp", "Kd"):
            for name, val in values.items():
                self._gains[name][data_name.lower()] = float(val)
        elif data_name == "Goal_Position":
            for name, val in values.items():
                self._send_goal(name, float(val))
        else:
            raise ValueError(f"sync_write '{data_name}' not supported by {self.__class__.__name__}")

    def _send_goal(self, name: str, position_deg: float) -> None:
        motor_id = self.motors[name].id
        kp = self._gains[name]["kp"]
        kd = self._gains[name]["kd"]
        tau_raw, data = encode_motion_control(position_deg, 0.0, kp, kd, 0.0, self._ranges[name])
        self._send(build_extended_can_id(COMM_MOTION_CONTROL, tau_raw, motor_id), data)

    def _cached(self, data_name: str, motor: str) -> Value:
        with self._lock:
            st = dict(self._state[motor])
        mapping = {
            "Present_Position": st["position"],
            "Present_Velocity": st["velocity"],
            "Present_Torque": st["torque"],
            "Temperature": st["temperature"],
        }
        if data_name not in mapping:
            raise ValueError(f"Unknown data_name: {data_name}")
        return mapping[data_name]

    # ------------------------------------------------------------------ calibration
    def read_calibration(self) -> dict[str, MotorCalibration]:
        return self.calibration if self.calibration else {}

    def write_calibration(self, calibration_dict: dict[str, MotorCalibration], cache: bool = True) -> None:
        if cache:
            self.calibration = calibration_dict

    @property
    def is_calibrated(self) -> bool:
        return bool(self.calibration)

    # ------------------------------------------------------------------ utils
    def _names(self, motors: str | list[str] | None) -> list[str]:
        if motors is None:
            return list(self.motors.keys())
        if isinstance(motors, str):
            return [motors]
        return list(motors)
