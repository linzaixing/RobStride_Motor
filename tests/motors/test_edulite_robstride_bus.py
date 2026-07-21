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

"""Byte-level tests for the EDULITE private-protocol CAN encoding.

These validate that the LeRobot ``EduliteRobstrideBus`` produces frames identical
to the upstream EDULITE_A3 SDK (``el_a3_sdk/el_a3_sdk/can_driver.py``), i.e. that a
message put on the wire by lerobot would be understood byte-for-byte by the motors.
No CAN hardware or socket is required.
"""

import struct

import numpy as np
import pytest

from lerobot.robots.edulite_a3_follower.edulite_can_bus import (
    _MODEL_RANGES,
    CAN_EFF_FLAG,
    build_extended_can_id,
    decode_feedback,
    encode_motion_control,
    float_to_uint16,
    uint16_to_float,
)


def test_float_uint16_roundtrip():
    for x in (-12.57, -3.0, 0.0, 3.0, 12.57):
        u = float_to_uint16(x, -12.57, 12.57)
        assert 0 <= u <= 65535
        assert uint16_to_float(u, -12.57, 12.57) == pytest.approx(x, abs=1e-3)
    # clamping
    assert float_to_uint16(100.0, -12.57, 12.57) == 65535
    assert float_to_uint16(-100.0, -12.57, 12.57) == 0
    # midpoint maps to ~32767
    assert float_to_uint16(0.0, -12.57, 12.57) == pytest.approx(32767, abs=1)


def test_extended_can_id_layout():
    # motion control (type 1), feed-forward torque raw in bits 23..8, motor id in 7..0
    can_id = build_extended_can_id(1, 0xABCD, 0x03)
    assert can_id & CAN_EFF_FLAG  # extended-frame flag set
    raw = can_id & 0x1FFFFFFF
    assert (raw >> 24) & 0x1F == 1  # comm type
    assert (raw >> 8) & 0xFFFF == 0xABCD  # data area 2 (torque)
    assert raw & 0xFF == 0x03  # target id


def test_encode_motion_control_matches_edulite_formula():
    """Reproduce el_a3_sdk.can_driver.send_motion_control byte packing for an RS00."""
    ranges = _MODEL_RANGES["O0"]  # RS00: v +-33, t +-14
    pos_deg, vel_deg, kp, kd, tau = 30.0, 0.0, 10.0, 0.5, 0.0

    tau_raw, data = encode_motion_control(pos_deg, vel_deg, kp, kd, tau, ranges)

    # Independent reference computation (mirrors the SDK exactly).
    def f2u(x, lo, hi):
        x = max(lo, min(hi, x))
        return int((x - lo) * 65535.0 / (hi - lo))

    exp_pos = f2u(np.radians(30.0), -12.57, 12.57)
    exp_vel = f2u(0.0, -33.0, 33.0)
    exp_kp = f2u(10.0, 0.0, 500.0)
    exp_kd = f2u(0.5, 0.0, 5.0)
    exp_tau = f2u(0.0, -14.0, 14.0)

    assert data == struct.pack(">HHHH", exp_pos, exp_vel, exp_kp, exp_kd)
    assert tau_raw == exp_tau
    assert len(data) == 8


def test_feedback_decode_is_inverse_of_encode_position():
    ranges = _MODEL_RANGES["ELO5"]  # EL05
    # Build a synthetic type-2 feedback payload with a known position.
    pos_deg = 45.0
    pos_raw = float_to_uint16(np.radians(pos_deg), -12.57, 12.57)
    payload = struct.pack(">HHHH", pos_raw, 32767, 32767, 250)  # vel~0, tau~0, temp 25.0C
    decoded = decode_feedback(payload, ranges)
    assert decoded["position"] == pytest.approx(pos_deg, abs=0.05)
    assert decoded["velocity"] == pytest.approx(0.0, abs=0.05)
    assert decoded["temperature"] == pytest.approx(25.0, abs=0.001)


def test_model_ranges_cover_edulite_motors():
    assert _MODEL_RANGES["O0"]["t_max"] == 14.0  # RS00
    assert _MODEL_RANGES["ELO5"]["t_max"] == 6.0  # EL05
    assert _MODEL_RANGES["O5"]["t_max"] == 5.5  # RS05
    for r in _MODEL_RANGES.values():
        assert r["v_max"] in (33.0, 50.0)
