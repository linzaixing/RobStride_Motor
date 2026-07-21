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

from unittest.mock import MagicMock, patch

import pytest

from lerobot.robots.edulite_a3_follower.edulite_a3_follower import (
    _JOINT_DIRECTIONS,
    _MOTOR_LAYOUT,
)
from lerobot.teleoperators.edulite_a3_leader import (
    EduliteA3Leader,
    EduliteA3LeaderTeleopConfig,
)

_MODULE = "lerobot.teleoperators.edulite_a3_leader.edulite_a3_leader"
_RAW_PRESENT = 5.0  # degrees reported raw by every motor in the mock


def _make_bus_mock() -> MagicMock:
    bus = MagicMock(name="RobstrideBusMock")
    bus.is_connected = False

    def _connect(*_a, **_k):
        bus.is_connected = True

    def _disconnect(_disable=True):
        bus.is_connected = False

    bus.connect.side_effect = _connect
    bus.disconnect.side_effect = _disconnect
    return bus


@pytest.fixture
def leader():
    bus_mock = _make_bus_mock()

    def _bus_side_effect(*_args, **kwargs):
        bus_mock.motors = kwargs["motors"]
        bus_mock.sync_read.return_value = dict.fromkeys(bus_mock.motors, _RAW_PRESENT)
        bus_mock.sync_write.return_value = None
        bus_mock.write.return_value = None
        return bus_mock

    with patch(f"{_MODULE}.RobstrideMotorsBus", side_effect=_bus_side_effect):
        cfg = EduliteA3LeaderTeleopConfig(port="can0", protocol="mit")
        teleop = EduliteA3Leader(cfg)
        teleop.connect(calibrate=False)
        yield teleop
        if teleop.is_connected:
            teleop.disconnect()


def test_action_features_match_joints(leader):
    assert set(leader.action_features) == {f"{j}.pos" for j in _MOTOR_LAYOUT}
    assert leader.feedback_features == leader.action_features


def test_connect_disconnect(leader):
    assert leader.is_connected
    leader.disconnect()
    assert not leader.is_connected


def test_get_action_applies_direction_signs(leader):
    action = leader.get_action()
    assert set(action) == {f"{j}.pos" for j in _MOTOR_LAYOUT}
    # logical = raw * direction; L1 dir=-1 so +5 raw -> -5 logical.
    assert action["L1.pos"] == pytest.approx(-_RAW_PRESENT)
    assert action["L2.pos"] == pytest.approx(_RAW_PRESENT)
    for joint in _MOTOR_LAYOUT:
        assert action[f"{joint}.pos"] == pytest.approx(_RAW_PRESENT * _JOINT_DIRECTIONS[joint])


def test_configure_zero_torque_damping_sets_kp0_kd():
    """With use_zero_torque_damping=True, configure must set kp=0 and kd on every joint."""
    bus_mock = _make_bus_mock()

    def _bus_side_effect(*_args, **kwargs):
        bus_mock.motors = kwargs["motors"]
        return bus_mock

    with patch(f"{_MODULE}.RobstrideMotorsBus", side_effect=_bus_side_effect):
        cfg = EduliteA3LeaderTeleopConfig(port="can0", protocol="mit", use_zero_torque_damping=True, kd=0.3)
        teleop = EduliteA3Leader(cfg)
        teleop.connect(calibrate=False)

    bus_mock.configure_motors.assert_called_once()
    kp_calls = [c for c in bus_mock.write.call_args_list if c.args[0] == "Kp"]
    kd_calls = [c for c in bus_mock.write.call_args_list if c.args[0] == "Kd"]
    assert len(kp_calls) == 7 and all(c.args[2] == 0.0 for c in kp_calls)
    assert len(kd_calls) == 7 and all(c.args[2] == 0.3 for c in kd_calls)


def test_configure_full_free_disables_torque():
    """With use_zero_torque_damping=False, configure must fully disable torque."""
    bus_mock = _make_bus_mock()

    def _bus_side_effect(*_args, **kwargs):
        bus_mock.motors = kwargs["motors"]
        return bus_mock

    with patch(f"{_MODULE}.RobstrideMotorsBus", side_effect=_bus_side_effect):
        cfg = EduliteA3LeaderTeleopConfig(port="can0", protocol="mit", use_zero_torque_damping=False)
        teleop = EduliteA3Leader(cfg)
        teleop.connect(calibrate=False)

    bus_mock.disable_torque.assert_called()
    bus_mock.configure_motors.assert_not_called()


def test_send_feedback_direction_roundtrip(leader):
    feedback = {f"{j}.pos": 12.0 for j in _MOTOR_LAYOUT}
    leader.send_feedback(feedback)
    args, _ = leader.bus.sync_write.call_args
    assert args[0] == "Goal_Position"
    for joint in _MOTOR_LAYOUT:
        assert args[1][joint] == pytest.approx(12.0 * _JOINT_DIRECTIONS[joint])
