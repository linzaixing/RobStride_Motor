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

"""End-to-end integration test for the EDULITE-A3 adapter over the private-protocol bus.

A fake SocketCAN socket emulates the 7 RobStride motors: it decodes the Type-1
motion-control frames produced by the real ``EduliteRobstrideBus``, integrates each
joint toward its target, and replies with Type-2 feedback frames. This drives the full
stack (``EduliteA3Follower`` -> ``EduliteRobstrideBus`` -> rx thread) with no hardware.
"""

import queue
import struct
import threading

import pytest

from lerobot.robots.edulite_a3_follower import (
    EduliteA3Follower,
    EduliteA3FollowerRobotConfig,
    edulite_can_bus as ecb,
)
from lerobot.robots.edulite_a3_follower.edulite_a3_follower import _MOTOR_LAYOUT

_STEP_FRAC = 0.4  # fraction of remaining distance a joint closes per motion command


class _FakeCanSocket:
    """Emulates 7 EDULITE motors speaking the RobStride private protocol."""

    def __init__(self, *_a, **_k):
        self.timeout = 0.1
        self.pos_rad = {mid: 0.0 for _, (mid, _m) in _MOTOR_LAYOUT.items()}
        self.model = {mid: model for _, (mid, model) in _MOTOR_LAYOUT.items()}
        self._rx: queue.Queue[bytes] = queue.Queue()
        self._lock = threading.Lock()

    def bind(self, *_a):
        pass

    def setsockopt(self, *_a):
        pass

    def settimeout(self, t):
        self.timeout = t

    def close(self):
        pass

    def send(self, frame: bytes):
        can_id_raw, _dlc, data = struct.unpack(ecb.CAN_FRAME_FMT, frame)
        can_id = can_id_raw & ecb.CAN_EFF_MASK
        comm = (can_id >> 24) & 0x1F
        mid = can_id & 0xFF
        if mid not in self.pos_rad:
            return
        if comm == ecb.COMM_SET_ZERO:
            with self._lock:
                self.pos_rad[mid] = 0.0
            self._emit(mid)
        elif comm == ecb.COMM_MOTION_CONTROL:
            pos_raw, _v, kp_raw, _kd = struct.unpack(">HHHH", data)
            target = ecb.uint16_to_float(pos_raw, ecb._P_MIN, ecb._P_MAX)
            with self._lock:
                if kp_raw > 0:  # real goal; kp==0 is a feedback-only poll
                    self.pos_rad[mid] += _STEP_FRAC * (target - self.pos_rad[mid])
            self._emit(mid)

    def recv(self, _n):
        try:
            return self._rx.get(timeout=self.timeout)
        except queue.Empty as e:
            raise TimeoutError from e

    def _emit(self, mid: int):
        r = ecb._MODEL_RANGES[self.model[mid]]
        with self._lock:
            pos = self.pos_rad[mid]
        data = struct.pack(
            ">HHHH",
            ecb.float_to_uint16(pos, ecb._P_MIN, ecb._P_MAX),
            ecb.float_to_uint16(0.0, r["v_min"], r["v_max"]),
            ecb.float_to_uint16(0.0, r["t_min"], r["t_max"]),
            250,  # 25.0 C
        )
        # motor id travels in data-area-2 (bits 15..8), matching the bus decoder.
        arb = ecb.build_extended_can_id(ecb.COMM_FEEDBACK, mid, 0xFD)
        self._rx.put(struct.pack(ecb.CAN_FRAME_FMT, arb, 8, data))


@pytest.fixture
def sim_robot(monkeypatch):
    fake = _FakeCanSocket()
    monkeypatch.setattr(ecb.socket, "socket", lambda *a, **k: fake)
    robot = EduliteA3Follower(EduliteA3FollowerRobotConfig(port="vcan0", protocol="private"))
    robot.connect(calibrate=False)
    yield robot, fake
    if robot.is_connected:
        robot.disconnect()


def test_connects_over_private_bus(sim_robot):
    robot, _ = sim_robot
    assert robot.is_connected
    assert type(robot.bus).__name__ == "EduliteRobstrideBus"


def test_closed_loop_converges_to_target(sim_robot):
    robot, _ = sim_robot
    target = {
        "L1.pos": 40.0, "L2.pos": 60.0, "L3.pos": -50.0,
        "L4.pos": 30.0, "L5.pos": -25.0, "L6.pos": 20.0, "L7.pos": 15.0,
    }
    obs = {}
    for _ in range(30):
        robot.send_action(target)
        obs = robot.get_observation()

    for joint in _MOTOR_LAYOUT:
        assert obs[f"{joint}.pos"] == pytest.approx(target[f"{joint}.pos"], abs=0.5)


def test_observation_has_all_joints(sim_robot):
    robot, _ = sim_robot
    obs = robot.get_observation()
    assert {f"{j}.pos" for j in _MOTOR_LAYOUT} <= set(obs)


def test_limits_are_enforced_end_to_end(sim_robot):
    robot, _ = sim_robot
    # Command way past L3's lower limit (-230 deg) and L2's upper limit (210 deg).
    over = {f"{j}.pos": 9999.0 for j in _MOTOR_LAYOUT}
    over["L3.pos"] = -9999.0
    returned = robot.send_action(over)
    assert returned["L2.pos"] == pytest.approx(210.0)
    assert returned["L3.pos"] == pytest.approx(-230.0)
    assert returned["L4.pos"] == pytest.approx(90.0)
