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

"""Scripted IK 'expert' for the EDULITE-A3 pick-and-place env.

Generates a demonstration trajectory (approach -> descend -> close gripper -> lift ->
transfer -> descend -> open gripper -> retract) by solving Pinocchio position-IK for
the end-effector and emitting joint-angle actions in the env's logical-degree frame.
Grasping itself is handled by the environment's weld constraint (triggered when the
gripper is closed near the ball), so this expert only decides *where* to move and
*when* to open/close the gripper -- it never teleports the ball.

Used by ``demo_pick_place.py`` and ``pick_place_benchmark.py``.
"""

from pathlib import Path

import numpy as np
import pinocchio as pin

_URDF = Path(__file__).resolve().parents[2] / "src/lerobot/envs/edulite/assets/el_a3.urdf"
JOINT_DIRECTIONS = np.array([-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0])
GRIP_OPEN, GRIP_CLOSE = -60.0, 60.0  # L7 logical degrees


class PinocchioIK:
    def __init__(self, urdf: str | Path = _URDF):
        self.m = pin.buildModelFromUrdf(str(urdf))
        self.d = pin.Data(self.m)
        self.ee = self.m.getFrameId("end_effector")

    def solve(self, target_xyz, q0=None, iters=300):
        q = np.zeros(self.m.nq) if q0 is None else np.array(q0, dtype=float)
        for _ in range(iters):
            pin.forwardKinematics(self.m, self.d, q)
            pin.updateFramePlacements(self.m, self.d)
            err = np.asarray(target_xyz) - self.d.oMf[self.ee].translation
            if np.linalg.norm(err) < 1e-3:
                break
            J = pin.computeFrameJacobian(self.m, self.d, q, self.ee, pin.LOCAL_WORLD_ALIGNED)[:3]  # noqa: N806
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(3), err)
            q = np.clip(q + 0.5 * dq, self.m.lowerPositionLimit, self.m.upperPositionLimit)
        pin.forwardKinematics(self.m, self.d, q)
        pin.updateFramePlacements(self.m, self.d)
        residual = float(np.linalg.norm(np.asarray(target_xyz) - self.d.oMf[self.ee].translation))
        return q, residual

    @staticmethod
    def motor_rad_to_logical_deg(q_rad):
        return (np.degrees(q_rad[:7]) * JOINT_DIRECTIONS).astype(np.float32)


class ScriptedPickPlace:
    """Phase-machine expert. Call ``act(ee, ball, target)`` each env step."""

    # gripper_link -> jaw-hull-center offset (world, approx top-down approach): the IK
    # targets are shifted by this so the real jaw (not gripper_link) reaches the object.
    JAW_OFFSET = np.array([0.039, -0.038, -0.041])

    def __init__(self, ik: PinocchioIK, lift_h=0.14, approach_h=0.14, steps_per_phase=None):
        self.ik = ik
        self._q = np.zeros(ik.m.nq)
        self._phase = 0
        self._t = 0
        self.lift_h = lift_h
        self.approach_h = approach_h
        # (name, gripper_closed, n_steps)
        self.plan = steps_per_phase or [
            ("approach", False, 36),
            ("descend", False, 26),
            ("grasp", True, 16),
            ("lift", True, 30),
            ("transfer", True, 60),
            ("place_down", True, 80),
            ("release", False, 20),
            ("retract", False, 30),
        ]
        self.done = False
        self._grasp_offset = None

    def _phase_target(self, name, ball, target):
        base = ball if name in ("approach", "descend", "grasp") else target
        # aim the jaw hull (offset from gripper_link) at the object
        base = np.asarray(base, dtype=float) - self.JAW_OFFSET
        z = base[2]
        if name in ("approach", "lift", "retract", "transfer"):
            z = base[2] + self.approach_h
        elif name == "descend":
            z = base[2] + 0.06  # hover just above the object (gripper open, no contact yet)
        elif name in ("grasp", "place_down", "release"):
            z = base[2] + 0.015  # lower onto the object; grasp phase closes -> contact
        return np.array([base[0], base[1], z])

    def act(self, ee, ball, target, jaw=None):
        if self._phase >= len(self.plan):
            self.done = True
            return self._last_action()
        name, gclosed, nsteps = self.plan[self._phase]
        ee = np.asarray(ee, dtype=float)
        ball = np.asarray(ball, dtype=float)
        target = np.asarray(target, dtype=float)
        jaw = ee if jaw is None else np.asarray(jaw, dtype=float)

        def _damped(delta, cap=0.03):
            n = float(np.linalg.norm(delta))
            return delta * (cap / n) if n > cap else delta

        # record the (ee - object) offset once the object is firmly held (start of lift),
        # then place using it so the object lands on the target regardless of grasp pose.
        if gclosed and name == "lift" and self._grasp_offset is None:
            self._grasp_offset = ee - ball

        if gclosed and name in ("lift", "transfer", "place_down") and self._grasp_offset is not None:
            # damped closed loop: move the gripper so the *object* converges to the goal
            goal = target.copy()
            goal[2] = target[2] + (0.14 if name in ("lift", "transfer") else 0.02)
            tgt = ee + _damped(goal - ball, 0.03)
        elif name in ("approach", "descend", "grasp"):
            # orientation-agnostic closed loop: drive the JAW CENTER onto the object using
            # the actual jaw feedback (works for any wrist orientation, unlike a fixed offset)
            desired_jaw = ball.copy()
            desired_jaw[2] = ball[2] + {"approach": 0.13, "descend": 0.05, "grasp": 0.0}[name]
            tgt = ee + _damped(desired_jaw - jaw)
        elif name in ("release", "retract"):
            # lift the (now open) gripper STRAIGHT UP so it clears the just-placed object
            # instead of sweeping sideways through it
            tgt = ee + np.array([0.0, 0.0, 0.035])
        else:
            tgt = self._phase_target(name, ball, target)

        # clamp the IK target into the reachable workspace so the arm never swings wildly
        # toward an unreachable point (which would fling a grasped object)
        tgt = np.asarray(tgt, dtype=float)
        r = float(np.linalg.norm(tgt[:2]))
        if r > 0.22:
            tgt[:2] *= 0.22 / r
        tgt[2] = float(np.clip(tgt[2], 0.03, 0.32))

        q, _ = self.ik.solve(tgt, self._q)
        self._q = q
        action = self.ik.motor_rad_to_logical_deg(q).copy()
        action[6] = GRIP_CLOSE if gclosed else GRIP_OPEN
        self._last = action
        self._t += 1
        if self._t >= nsteps:
            self._t = 0
            self._phase += 1
        return action

    def _last_action(self):
        a = getattr(self, "_last", np.zeros(7, dtype=np.float32)).copy()
        a[6] = GRIP_OPEN
        return a
