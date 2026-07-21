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

This bridges the open-source EDULITE_A3 hardware (https://github.com/RobStride/EDULITE_A3)
into the LeRobot ecosystem so it can be used for teleoperated data collection,
policy training and VLA inference, reusing lerobot's built-in ``RobstrideMotorsBus``.

Motor mapping (EDULITE joint -> CAN id -> robstride model):
    L1 -> 1 -> O0   (RS00)     L4 -> 4 -> ELO5 (EL05)
    L2 -> 2 -> O0   (RS00)     L5 -> 5 -> ELO5 (EL05)
    L3 -> 3 -> O0   (RS00)     L6 -> 6 -> ELO5 (EL05)
                               L7 -> 7 -> ELO5 (EL05, gripper)
"""

import logging
import time
from functools import cached_property

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

# EDULITE joint name -> (CAN id, robstride model string)
# Mirrors DEFAULT_MOTOR_TYPE_MAP in el_a3_sdk/el_a3_sdk/protocol.py
_MOTOR_LAYOUT: dict[str, tuple[int, str]] = {
    "L1": (1, "O0"),
    "L2": (2, "O0"),
    "L3": (3, "O0"),
    "L4": (4, "ELO5"),
    "L5": (5, "ELO5"),
    "L6": (6, "ELO5"),
    "L7": (7, "ELO5"),  # gripper
}

# Per-joint sign convention (logical joint frame vs. raw motor frame).
# Mirrors DEFAULT_JOINT_DIRECTIONS in el_a3_sdk/el_a3_sdk/protocol.py
_JOINT_DIRECTIONS: dict[str, float] = {
    "L1": -1.0,
    "L2": 1.0,
    "L3": -1.0,
    "L4": 1.0,
    "L5": -1.0,
    "L6": 1.0,
    "L7": 1.0,
}

# Soft joint limits in degrees (mirrors DEFAULT_JOINT_LIMITS, rad -> deg).
_JOINT_LIMITS_DEG: dict[str, tuple[float, float]] = {
    "L1": (-160.0, 160.0),
    "L2": (0.0, 210.0),
    "L3": (-230.0, 0.0),
    "L4": (-90.0, 90.0),
    "L5": (-90.0, 90.0),
    "L6": (-90.0, 90.0),
    "L7": (-90.0, 90.0),
}

GRIPPER = "L7"


class EduliteA3Follower(Robot):
    """EDULITE-A3 7-DOF arm exposed as a LeRobot follower robot."""

    config_class = EduliteA3FollowerRobotConfig
    name = "edulite_a3_follower"

    def __init__(self, config: EduliteA3FollowerRobotConfig):
        super().__init__(config)
        self.config = config

        norm_mode = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        motors = {}
        for joint, (can_id, model) in _MOTOR_LAYOUT.items():
            motors[joint] = Motor(
                id=can_id,
                model=model,
                norm_mode=norm_mode,
                motor_type_str=model,
                recv_id=can_id,
            )

        bus_cls = EduliteRobstrideBus if config.protocol == "private" else RobstrideMotorsBus
        self.bus = bus_cls(
            port=config.port,
            motors=motors,
            calibration=self.calibration,
            can_interface=config.can_interface,
            use_can_fd=config.use_can_fd,
            bitrate=config.bitrate,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

    # ------------------------------------------------------------------ features
    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in self.bus.motors}

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
        return self._motors_ft

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
        """Zero the motors at the current pose.

        Move the arm to its mechanical/home reference (all joints at their SDK zero
        position) before calling, then the current pose is stored as the electrical
        zero for every joint.
        """
        input(f"Move {self} to its home (zero) pose and press ENTER to set zero...")
        self.bus.disable_torque()
        self.bus.set_zero_position()
        # Record nominal soft limits for bookkeeping / dataset metadata.
        self.calibration = {}
        for joint, m in self.bus.motors.items():
            lo, hi = _JOINT_LIMITS_DEG[joint]
            self.calibration[joint] = MotorCalibration(
                id=m.id,
                drive_mode=0 if _JOINT_DIRECTIONS[joint] > 0 else 1,
                homing_offset=0,
                range_min=int(lo),
                range_max=int(hi),
            )
        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        logger.info(f"{self} zeroed; calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        """Enable all motors in MIT mode and set default position gains."""
        self.bus.configure_motors()  # enable + switch to MIT mode
        for joint in self.bus.motors:
            self.bus.write("Kp", joint, self.config.kp)
            self.bus.write("Kd", joint, self.config.kd)

    # ------------------------------------------------------------------ io helpers
    def _motor_to_logical(self, joint: str, motor_deg: float) -> float:
        return motor_deg * _JOINT_DIRECTIONS[joint]

    def _logical_to_motor(self, joint: str, logical_deg: float) -> float:
        return logical_deg * _JOINT_DIRECTIONS[joint]

    # ------------------------------------------------------------------ observation
    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()
        raw = self.bus.sync_read("Present_Position")
        obs_dict = {f"{joint}.pos": self._motor_to_logical(joint, val) for joint, val in raw.items()}
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

        # Clamp each goal to the joint's soft limits (degrees, logical frame).
        for joint, val in list(goal_logical.items()):
            lo, hi = _JOINT_LIMITS_DEG.get(joint, (-360.0, 360.0))
            goal_logical[joint] = max(lo, min(hi, val))

        # Optional relative-motion safety cap (compares in logical frame).
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
