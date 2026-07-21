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

from dataclasses import dataclass

from ..config import TeleoperatorConfig


@dataclass
class EduliteA3LeaderConfig:
    """Configuration for the EDULITE-A3 arm used as a kinesthetic-teaching leader.

    The same RobStride 7-DOF arm acts as the teleoperator: torque is disabled (or a
    light gravity-compensation kd is applied) so an operator can drag the arm by hand
    while ``get_action`` streams the joint positions that a follower should mirror.
    """

    # CAN interface the leader arm is wired to (e.g. "can0").
    port: str = "can0"

    bitrate: int = 1_000_000
    use_can_fd: bool = False
    can_interface: str = "auto"

    # "private" (RobStride private protocol, EDULITE-native) or "mit" (lerobot built-in).
    protocol: str = "private"

    # If True, keep a small damping (kd) with kp=0 for smooth gravity-comp-style drag.
    # If False, fully disable torque so the arm is completely free.
    use_zero_torque_damping: bool = True
    kd: float = 0.3

    use_degrees: bool = True


@TeleoperatorConfig.register_subclass("edulite_a3_leader")
@dataclass
class EduliteA3LeaderTeleopConfig(TeleoperatorConfig, EduliteA3LeaderConfig):
    pass
