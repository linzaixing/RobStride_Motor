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
class EduliteA3FollowerConfig:
    """Configuration for the EDULITE-A3 7-DOF desktop arm (RobStride, CAN bus).

    The arm uses 7 RobStride integrated joint motors over a single 1 Mbps CAN 2.0 bus:
      * L1-L3 : RS00  (14 N.m)  -> lerobot robstride model "O0"
      * L4-L7 : EL05  (6 N.m)   -> lerobot robstride model "ELO5"  (L7 is the gripper)

    See the upstream EDULITE_A3 SDK ``el_a3_sdk/el_a3_sdk/protocol.py`` for the
    canonical joint directions / limits that these defaults mirror.
    """

    # CAN interface the arm is wired to (e.g. "can0"). For a serial/slcan adapter on
    # macOS use the "/dev/..." device path and the bus will auto-select slcan.
    port: str = "can0"

    # EDULITE-A3 uses classic CAN 2.0 @ 1 Mbps (NOT CAN FD).
    bitrate: int = 1_000_000
    use_can_fd: bool = False
    can_interface: str = "auto"

    # Wire protocol used to talk to the RobStride motors:
    #   "mit"     -> lerobot's built-in RobstrideMotorsBus (motors must be in MIT mode)
    #   "private" -> EduliteRobstrideBus, RobStride private protocol / 29-bit extended
    #                frames, byte-compatible with the upstream EDULITE_A3 SDK (default,
    #                works with motors left in their factory private-protocol mode).
    protocol: str = "private"

    disable_torque_on_disconnect: bool = True

    # `max_relative_target` limits the magnitude of the relative positional target
    # vector for safety. Scalar => same for all joints, or a per-joint dict.
    max_relative_target: float | dict[str, float] | None = None

    # MIT control gains applied to every joint at configure() time.
    # Lower kp = softer / safer, higher kp = stiffer tracking.
    kp: float = 10.0
    kd: float = 0.5

    # cameras (required for any VLA use: the "V" in Vision-Language-Action)
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Keep True to command/report joint values in degrees (matches robstride bus).
    use_degrees: bool = True


@RobotConfig.register_subclass("edulite_a3_follower")
@dataclass
class EduliteA3FollowerRobotConfig(RobotConfig, EduliteA3FollowerConfig):
    pass
