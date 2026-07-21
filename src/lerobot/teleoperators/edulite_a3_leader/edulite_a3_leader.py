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

"""EDULITE-A3 arm as a LeRobot leader teleoperator (kinesthetic / drag teaching).

The arm's torque is disabled (optionally with a light damping kd for a smooth,
gravity-compensation-like feel), letting an operator move it by hand. ``get_action``
returns the current joint positions (degrees, logical joint frame) so a follower arm
or the recording pipeline can mirror them. This turns EDULITE's native zero-torque
drag teaching into a lerobot data-collection source.
"""

import logging
import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.robstride import RobstrideMotorsBus
from lerobot.robots.edulite_a3_follower.edulite_a3_follower import (
    _JOINT_DIRECTIONS,
    _MOTOR_LAYOUT,
)
from lerobot.robots.edulite_a3_follower.edulite_can_bus import EduliteRobstrideBus
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_edulite_a3_leader import EduliteA3LeaderTeleopConfig

logger = logging.getLogger(__name__)


class EduliteA3Leader(Teleoperator):
    """EDULITE-A3 7-DOF arm exposed as a drag-teaching leader teleoperator."""

    config_class = EduliteA3LeaderTeleopConfig
    name = "edulite_a3_leader"

    def __init__(self, config: EduliteA3LeaderTeleopConfig):
        super().__init__(config)
        self.config = config

        norm_mode = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        motors = {
            joint: Motor(
                id=can_id,
                model=model,
                norm_mode=norm_mode,
                motor_type_str=model,
                recv_id=can_id,
            )
            for joint, (can_id, model) in _MOTOR_LAYOUT.items()
        }
        self.bus = (EduliteRobstrideBus if config.protocol == "private" else RobstrideMotorsBus)(
            port=config.port,
            motors=motors,
            calibration=self.calibration,
            can_interface=config.can_interface,
            use_can_fd=config.use_can_fd,
            bitrate=config.bitrate,
        )

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return self.action_features

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        self.configure()
        logger.info(f"{self} connected (drag-teaching mode).")

    @property
    def is_calibrated(self) -> bool:
        # RobStride motors have no internal calibration store; treat as always ready.
        return True

    def calibrate(self) -> None:
        # No-op: zeroing is handled on the follower side via set_zero_position.
        pass

    def configure(self) -> None:
        """Put the arm into a hand-movable state.

        Either fully disable torque, or keep kp=0 with a small kd for smooth,
        gravity-compensation-like back-drivability.
        """
        if self.config.use_zero_torque_damping:
            self.bus.configure_motors()  # enable + MIT mode
            for joint in self.bus.motors:
                self.bus.write("Kp", joint, 0.0)
                self.bus.write("Kd", joint, self.config.kd)
        else:
            self.bus.disable_torque()

    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()
        raw = self.bus.sync_read("Present_Position")
        action = {f"{joint}.pos": val * _JOINT_DIRECTIONS[joint] for joint, val in raw.items()}
        logger.debug(f"{self} read action: {(time.perf_counter() - start) * 1e3:.1f}ms")
        return action

    @check_if_not_connected
    def send_feedback(self, feedback: dict[str, float]) -> None:
        """Optional force/position feedback (e.g. bilateral master-slave).

        Commands the leader back toward `feedback` joint positions. Only effective
        when torque is enabled (use_zero_torque_damping with a non-zero kp set later).
        """
        goals = {
            joint.removesuffix(".pos"): val * _JOINT_DIRECTIONS[joint.removesuffix(".pos")]
            for joint, val in feedback.items()
            if joint.endswith(".pos")
        }
        if goals:
            self.bus.sync_write("Goal_Position", goals)

    @check_if_not_connected
    def disconnect(self) -> None:
        self.bus.disconnect()
        logger.info(f"{self} disconnected.")
