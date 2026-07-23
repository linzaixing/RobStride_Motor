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

"""LeRobot ``Robot`` adapter for the EDULITE-A3 7-DOF desktop arm (RobStride, CAN).

Bridges the open-source EDULITE_A3 hardware (https://github.com/RobStride/EDULITE_A3)
into the LeRobot ecosystem. It drives RobStride motors the same idiomatic way the
OpenArm follower drives Damiao motors: a ``motor_config`` (send/recv CAN id + model)
declares the motors on a native ``MotorsBus``, MIT position control uses per-joint
``position_kp/kd`` gains, and actions are clipped to ``joint_limits``.

Two wire protocols are supported via ``config.protocol``:
    "private" -> EduliteRobstrideBus (RobStride private protocol, 29-bit extended
                 frames; byte-compatible with the EDULITE_A3 SDK) — default.
    "mit"     -> lerobot's built-in RobstrideMotorsBus (motors in MIT mode).

Motor mapping (joint -> CAN id -> model): L1-L3 -> RS00 ("O0"), L4-L7 -> EL05 ("ELO5").
"""

import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.robstride import RobstrideMotorsBus
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_not_connected

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_edulite_a3_follower import EduliteA3FollowerRobotConfig
from .edulite_can_bus import EduliteRobstrideBus

logger = logging.getLogger(__name__)

GRIPPER = "L7"

# Per-joint sign convention (logical joint frame vs. raw motor frame).
# Mirrors DEFAULT_JOINT_DIRECTIONS in el_a3_sdk/el_a3_sdk/protocol.py.
JOINT_DIRECTIONS: dict[str, float] = {
    "L1": -1.0, "L2": 1.0, "L3": -1.0, "L4": 1.0, "L5": -1.0, "L6": 1.0, "L7": 1.0,
}

# --- backwards-compatible module constants (used by the leader, sim env and tests) ---
_JOINT_DIRECTIONS = JOINT_DIRECTIONS
_MOTOR_LAYOUT: dict[str, tuple[int, str]] = {
    "L1": (1, "O0"), "L2": (2, "O0"), "L3": (3, "O0"),
    "L4": (4, "ELO5"), "L5": (5, "ELO5"), "L6": (6, "ELO5"), "L7": (7, "ELO5"),
}
_JOINT_LIMITS_DEG: dict[str, tuple[float, float]] = {
    "L1": (-160.0, 160.0), "L2": (0.0, 210.0), "L3": (-230.0, 0.0),
    "L4": (-90.0, 90.0), "L5": (-90.0, 90.0), "L6": (-90.0, 90.0), "L7": (-90.0, 90.0),
}


class EduliteA3Follower(Robot):
    """EDULITE-A3 7-DOF arm exposed as a LeRobot follower robot."""

    config_class = EduliteA3FollowerRobotConfig
    name = "edulite_a3_follower"

    def __init__(self, config: EduliteA3FollowerRobotConfig):
        super().__init__(config)
        self.config = config

        # Build motors from motor_config, OpenArm/Damiao-style:
        # name -> (send_can_id, recv_can_id, robstride_model).
        norm_mode = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        motors: dict[str, Motor] = {}
        self._order: list[str] = list(config.motor_config.keys())
        for name, (send_id, recv_id, model) in config.motor_config.items():
            motor = Motor(id=send_id, model=model, norm_mode=norm_mode, motor_type_str=model)
            motor.recv_id = recv_id
            motors[name] = motor

        bus_cls = EduliteRobstrideBus if config.protocol == "private" else RobstrideMotorsBus
        self.bus = bus_cls(
            port=config.port,
            motors=motors,
            calibration=self.calibration,
            can_interface=config.can_interface,
            use_can_fd=config.use_can_fd,
            bitrate=config.can_bitrate,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

    # ------------------------------------------------------------------ features
    @property
    def _motors_ft(self) -> dict[str, type]:
        features: dict[str, type] = {}
        for motor in self.bus.motors:
            features[f"{motor}.pos"] = float
            if self.config.use_velocity_and_torque:
                features[f"{motor}.vel"] = float
                features[f"{motor}.torque"] = float
        return features

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        features: dict[str, tuple] = {}
        for cam_key, cam in self.cameras.items():
            if getattr(cam, "use_rgb", True):
                features[cam_key] = (cam.height, cam.width, 3)
            if getattr(cam, "use_depth", False):
                features[f"{cam_key}_depth"] = (cam.height, cam.width, 1)
        return features

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    # ------------------------------------------------------------------ gains
    def _gains(self, joint: str) -> tuple[float, float]:
        idx = self._order.index(joint)
        kp = self.config.position_kp
        kd = self.config.position_kd
        kp_v = kp[idx] if isinstance(kp, list) else kp
        kd_v = kd[idx] if isinstance(kd, list) else kd
        return float(kp_v), float(kd_v)

    # ------------------------------------------------------------------ lifecycle
    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()
        for cam in self.cameras.values():
            cam.connect()
        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        # RobStride motors don't store calibration internally; they are zeroed
        # electrically via set_zero_position, so we treat the arm as always usable.
        return True

    def calibrate(self) -> None:
        """Zero the motors at the current (home) pose."""
        input(f"Move {self} to its home (zero) pose and press ENTER to set zero...")
        self.bus.disable_torque()
        self.bus.set_zero_position()
        self.calibration = {}
        for joint, m in self.bus.motors.items():
            lo, hi = self.config.joint_limits.get(joint, (-90.0, 90.0))
            self.calibration[joint] = MotorCalibration(
                id=m.id,
                drive_mode=0 if JOINT_DIRECTIONS.get(joint, 1.0) > 0 else 1,
                homing_offset=0,
                range_min=int(lo),
                range_max=int(hi),
            )
        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        logger.info(f"{self} zeroed; calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        """Enable motors in MIT mode and apply per-joint position gains."""
        self.bus.configure_motors()  # enable + switch to MIT mode
        for joint in self.bus.motors:
            kp, kd = self._gains(joint)
            self.bus.write("Kp", joint, kp)
            self.bus.write("Kd", joint, kd)

    # ------------------------------------------------------------------ io helpers
    def _motor_to_logical(self, joint: str, motor_deg: float) -> float:
        return motor_deg * JOINT_DIRECTIONS.get(joint, 1.0)

    def _logical_to_motor(self, joint: str, logical_deg: float) -> float:
        return logical_deg * JOINT_DIRECTIONS.get(joint, 1.0)

    # ------------------------------------------------------------------ observation
    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()
        obs_dict: dict[str, Any] = {}
        pos = self.bus.sync_read("Present_Position")
        for joint in self.bus.motors:
            obs_dict[f"{joint}.pos"] = self._motor_to_logical(joint, pos[joint])
        if self.config.use_velocity_and_torque:
            vel = self.bus.sync_read("Present_Velocity")
            tau = self.bus.sync_read("Present_Torque")
            for joint in self.bus.motors:
                obs_dict[f"{joint}.vel"] = self._motor_to_logical(joint, vel[joint])
                obs_dict[f"{joint}.torque"] = tau[joint]
        logger.debug(f"{self} read state: {(time.perf_counter() - start) * 1e3:.1f}ms")

        for cam_key, cam in self.cameras.items():
            if getattr(cam, "use_rgb", True):
                obs_dict[cam_key] = cam.read_latest()
            if getattr(cam, "use_depth", False):
                obs_dict[f"{cam_key}_depth"] = cam.read_latest_depth()
        return obs_dict

    # ------------------------------------------------------------------ action
    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        goal_logical = {
            key.removesuffix(".pos"): float(val) for key, val in action.items() if key.endswith(".pos")
        }

        # Clip each goal to the joint's soft limits (degrees, logical frame).
        for joint, val in list(goal_logical.items()):
            lo, hi = self.config.joint_limits.get(joint, (-360.0, 360.0))
            goal_logical[joint] = max(lo, min(hi, val))

        # Optional relative-motion safety cap (compared in the logical frame).
        if self.config.max_relative_target is not None:
            raw_present = self.bus.sync_read("Present_Position")
            present_logical = {j: self._motor_to_logical(j, v) for j, v in raw_present.items()}
            goal_present = {j: (g, present_logical[j]) for j, g in goal_logical.items()}
            goal_logical = ensure_safe_goal_position(goal_present, self.config.max_relative_target)

        goal_motor = {j: self._logical_to_motor(j, v) for j, v in goal_logical.items()}
        self.bus.sync_write("Goal_Position", goal_motor)
        return {f"{joint}.pos": val for joint, val in goal_logical.items()}

    @check_if_not_connected
    def disconnect(self) -> None:
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()
        logger.info(f"{self} disconnected.")
