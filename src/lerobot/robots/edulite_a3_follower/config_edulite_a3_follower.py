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

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@dataclass
class EduliteA3FollowerConfigBase:
    """Configuration for the EDULITE-A3 7-DOF desktop arm (RobStride CAN motors).

    Structured after the OpenArm/Damiao follower so that RobStride motors are driven
    the same idiomatic way: a ``motor_config`` maps each joint to its CAN send/recv ID
    and RobStride model, and MIT position control uses per-joint ``position_kp/kd``.

    The 7 RobStride integrated joints on a single 1 Mbps CAN bus:
      * L1-L3 : RS00  (14 N.m)  -> robstride model "O0"
      * L4-L7 : EL05  (6 N.m)   -> robstride model "ELO5"  (L7 is the gripper)
    """

    # CAN interface the arm is wired to (e.g. "can0"); "/dev/..." selects slcan.
    port: str = "can0"

    # CAN interface type: "socketcan" (Linux), "slcan" (serial) or "auto".
    can_interface: str = "auto"

    # EDULITE-A3 uses classic CAN 2.0 @ 1 Mbps (NOT CAN FD).
    use_can_fd: bool = False
    can_bitrate: int = 1_000_000
    can_data_bitrate: int = 5_000_000  # only used when use_can_fd=True

    # Wire protocol to talk to the RobStride motors:
    #   "private" -> EduliteRobstrideBus, RobStride private protocol / 29-bit extended
    #                frames, byte-compatible with the EDULITE_A3 SDK (default; motors
    #                keep their factory mode and remain usable by the native SDK/ROS).
    #   "mit"     -> lerobot's built-in RobstrideMotorsBus (motors must be in MIT mode).
    protocol: str = "private"

    disable_torque_on_disconnect: bool = True

    # Expose `.vel` and `.torque` per motor in the observation when True.
    use_velocity_and_torque: bool = False

    # Relative target safety cap (degrees). Scalar for all joints, or a per-joint dict.
    max_relative_target: float | dict[str, float] | None = None

    # Cameras (required for any VLA use: the "V" in Vision-Language-Action).
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Report/command joint values in degrees (matches the RobStride bus convention).
    use_degrees: bool = True

    # Motor configuration: joint name -> (send_can_id, recv_can_id, robstride_model).
    # Mirrors DEFAULT_MOTOR_TYPE_MAP in el_a3_sdk/el_a3_sdk/protocol.py.
    motor_config: dict[str, tuple[int, int, str]] = field(
        default_factory=lambda: {
            "L1": (1, 1, "O0"),  # RS00 - base
            "L2": (2, 2, "O0"),  # RS00 - shoulder
            "L3": (3, 3, "O0"),  # RS00 - elbow
            "L4": (4, 4, "ELO5"),  # EL05 - wrist 1
            "L5": (5, 5, "ELO5"),  # EL05 - wrist 2
            "L6": (6, 6, "ELO5"),  # EL05 - wrist 3
            "L7": (7, 7, "ELO5"),  # EL05 - gripper
        }
    )

    # MIT position-control gains, one per joint [L1..L7] (RS00 stiffer than EL05).
    position_kp: list[float] = field(default_factory=lambda: [12.0, 12.0, 12.0, 8.0, 8.0, 8.0, 6.0])
    position_kd: list[float] = field(default_factory=lambda: [1.0, 1.0, 1.0, 0.6, 0.6, 0.6, 0.5])

    # Soft joint limits in degrees (logical frame); every action is clipped to these.
    # Mirrors DEFAULT_JOINT_LIMITS in el_a3_sdk/el_a3_sdk/protocol.py.
    joint_limits: dict[str, tuple[float, float]] = field(
        default_factory=lambda: {
            "L1": (-160.0, 160.0),
            "L2": (0.0, 210.0),
            "L3": (-230.0, 0.0),
            "L4": (-90.0, 90.0),
            "L5": (-90.0, 90.0),
            "L6": (-90.0, 90.0),
            "L7": (-90.0, 90.0),
        }
    )


@RobotConfig.register_subclass("edulite_a3_follower")
@dataclass
class EduliteA3FollowerRobotConfig(RobotConfig, EduliteA3FollowerConfigBase):
    pass


# Backwards-compatible alias.
EduliteA3FollowerConfig = EduliteA3FollowerRobotConfig
