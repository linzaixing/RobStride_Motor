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

from lerobot.robots.edulite_a3_follower import (
    EduliteA3Follower,
    EduliteA3FollowerRobotConfig,
)
from lerobot.robots.edulite_a3_follower.edulite_a3_follower import (
    _JOINT_DIRECTIONS,
    _JOINT_LIMITS_DEG,
    _MOTOR_LAYOUT,
)

_RAW_PRESENT = 30.0  # degrees reported by every motor in the mock


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
def follower():
    bus_mock = _make_bus_mock()

    def _bus_side_effect(*_args, **kwargs):
        bus_mock.motors = kwargs["motors"]
        bus_mock.sync_read.return_value = dict.fromkeys(bus_mock.motors, _RAW_PRESENT)
        bus_mock.sync_write.return_value = None
        bus_mock.write.return_value = None
        bus_mock.is_calibrated = True
        return bus_mock

    with (
        patch(
            "lerobot.robots.edulite_a3_follower.edulite_a3_follower.RobstrideMotorsBus",
            side_effect=_bus_side_effect,
        ),
        patch.object(EduliteA3Follower, "configure", lambda self: None),
    ):
        cfg = EduliteA3FollowerRobotConfig(port="can0", protocol="mit")
        robot = EduliteA3Follower(cfg)
        yield robot
        if robot.is_connected:
            robot.disconnect()


def test_motor_layout_matches_edulite():
    # 7 joints, RS00 (O0) on L1-L3, EL05 (ELO5) on L4-L7.
    assert list(_MOTOR_LAYOUT) == ["L1", "L2", "L3", "L4", "L5", "L6", "L7"]
    assert [m[1] for m in _MOTOR_LAYOUT.values()] == ["O0", "O0", "O0", "ELO5", "ELO5", "ELO5", "ELO5"]
    assert [m[0] for m in _MOTOR_LAYOUT.values()] == [1, 2, 3, 4, 5, 6, 7]


def test_config_motor_config_openarm_idiom():
    """The config declares motors OpenArm-style: name -> (send_id, recv_id, model)."""
    cfg = EduliteA3FollowerRobotConfig(port="can0")
    assert list(cfg.motor_config) == ["L1", "L2", "L3", "L4", "L5", "L6", "L7"]
    assert cfg.motor_config["L1"] == (1, 1, "O0")
    assert cfg.motor_config["L7"] == (7, 7, "ELO5")
    # per-joint MIT gains, one per joint
    assert len(cfg.position_kp) == 7 and len(cfg.position_kd) == 7
    # RS00 joints are stiffer than the EL05 wrist/gripper joints
    assert cfg.position_kp[0] > cfg.position_kp[6]


def test_per_joint_gains_written_on_configure():
    """configure() must push each joint's own kp/kd from the config lists to the bus."""
    bus_mock = _make_bus_mock()

    def _bus_side_effect(*_args, **kwargs):
        bus_mock.motors = kwargs["motors"]
        bus_mock.sync_read.return_value = dict.fromkeys(bus_mock.motors, _RAW_PRESENT)
        return bus_mock

    with patch(
        "lerobot.robots.edulite_a3_follower.edulite_a3_follower.RobstrideMotorsBus",
        side_effect=_bus_side_effect,
    ):
        cfg = EduliteA3FollowerRobotConfig(port="can0", protocol="mit")
        robot = EduliteA3Follower(cfg)
        robot.connect()  # runs configure()

    kp_calls = {c.args[1]: c.args[2] for c in bus_mock.write.call_args_list if c.args[0] == "Kp"}
    kd_calls = {c.args[1]: c.args[2] for c in bus_mock.write.call_args_list if c.args[0] == "Kd"}
    assert kp_calls["L1"] == cfg.position_kp[0]
    assert kp_calls["L7"] == cfg.position_kp[6]
    assert kd_calls["L4"] == cfg.position_kd[3]



def test_connect_disconnect(follower):
    assert not follower.is_connected
    follower.connect()
    assert follower.is_connected
    follower.disconnect()
    assert not follower.is_connected


def test_features_have_seven_joints(follower):
    assert set(follower.action_features) == {f"{j}.pos" for j in _MOTOR_LAYOUT}
    assert len(follower.action_features) == 7


def test_get_observation_applies_direction_signs(follower):
    follower.connect()
    obs = follower.get_observation()
    # logical = raw * direction
    for joint in _MOTOR_LAYOUT:
        assert obs[f"{joint}.pos"] == pytest.approx(_RAW_PRESENT * _JOINT_DIRECTIONS[joint])


def test_send_action_direction_roundtrip(follower):
    follower.connect()
    # Command a logical target well inside every joint limit.
    logical_target = {f"{j}.pos": 20.0 for j in _MOTOR_LAYOUT}
    # L3 must be negative (limits -230..0); use a valid in-range value there.
    logical_target["L3.pos"] = -20.0

    returned = follower.send_action(logical_target)

    # Returned action is in the logical frame and unchanged (within limits).
    for j in _MOTOR_LAYOUT:
        assert returned[f"{j}.pos"] == pytest.approx(logical_target[f"{j}.pos"])

    # The bus receives motor-frame values = logical * direction.
    args, _ = follower.bus.sync_write.call_args
    assert args[0] == "Goal_Position"
    motor_goals = args[1]
    for j in _MOTOR_LAYOUT:
        assert motor_goals[j] == pytest.approx(logical_target[f"{j}.pos"] * _JOINT_DIRECTIONS[j])


def test_send_action_clamps_to_limits(follower):
    follower.connect()
    # Push every joint far beyond its range in both directions.
    huge = {f"{j}.pos": 10_000.0 for j in _MOTOR_LAYOUT}
    returned = follower.send_action(huge)
    for j in _MOTOR_LAYOUT:
        lo, hi = _JOINT_LIMITS_DEG[j]
        assert returned[f"{j}.pos"] == pytest.approx(hi)

    huge_neg = {f"{j}.pos": -10_000.0 for j in _MOTOR_LAYOUT}
    returned = follower.send_action(huge_neg)
    for j in _MOTOR_LAYOUT:
        lo, hi = _JOINT_LIMITS_DEG[j]
        assert returned[f"{j}.pos"] == pytest.approx(lo)
