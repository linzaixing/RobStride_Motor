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

"""Scripted MuJoCo-IK expert for the SO-101 pick-and-place env.

Generates joint-target actions (the env's action space) with a 6-DOF-aware IK:
position + approach-axis-down (2-DOF) so the 5-DOF arm can both reach and point down;
a dual seed (current + a bent ready pose) avoids local-minima stalls, and a high central
waypoint lets the base rotate freely between pick and place. Privileged: it reads the
env's MuJoCo model/data to solve IK. Used by demo_pick_place.py / pick_place_benchmark.py.
"""

import mujoco
import numpy as np

READY_SEED = [0.0, -1.4, 1.4, -1.2, 0.0]
GRIP_OPEN, GRIP_CLOSE = -90.0, 90.0  # action[5] gripper command (deg)


class So101IKExpert:
    def __init__(self, env):
        self.e = env

    def _ik_step(self, target, k_rot=0.5, gain=0.4):
        e = self.e
        mujoco.mj_forward(e.m, e.d)
        pos_err = target - e.pinch_xyz()
        Rc = e.d.site_xmat[e.sid].reshape(3, 3)  # noqa: N806
        approach_cur = -Rc[:, 1]                       # finger direction in world
        rot_err = np.cross(approach_cur, np.array([0.0, 0.0, -1.0]))  # point straight down
        jacp = np.zeros((3, e.m.nv))
        jacr = np.zeros((3, e.m.nv))
        mujoco.mj_jacSite(e.m, e.d, jacp, jacr, e.sid)
        arm_dof = [e.m.jnt_dofadr[mujoco.mj_name2id(e.m, mujoco.mjtObj.mjOBJ_JOINT, n)]
                   for n in ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"]]
        J = np.vstack([jacp[:, arm_dof], jacr[:, arm_dof]])  # noqa: N806
        err = np.concatenate([pos_err, k_rot * rot_err])
        return gain * (J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), err))

    def _solve(self, target, seed):
        e = self.e
        save = e.d.qpos.copy()
        if seed is not None:
            for i, v in enumerate(seed):
                e.d.qpos[e.arm_q[i]] = v
        mujoco.mj_forward(e.m, e.d)
        for _ in range(300):
            dq = self._ik_step(target)
            e.d.qpos[e.arm_q] = np.clip(e.d.qpos[e.arm_q] + dq, e.arm_range[:, 0], e.arm_range[:, 1])
            if np.linalg.norm(dq) < 1e-4:
                break
        q = e.d.qpos[e.arm_q].copy()
        res = float(np.linalg.norm(target - e.pinch_xyz()))
        e.d.qpos[:] = save
        mujoco.mj_forward(e.m, e.d)
        return q, res

    def joint_target_deg(self, cart_target):
        """Best arm joint target (deg) reaching the Cartesian target, dual-seeded IK."""
        q_cur, r_cur = self._solve(cart_target, None)
        q_seed, r_seed = self._solve(cart_target, READY_SEED)
        q = q_cur if r_cur <= r_seed + 1e-3 else q_seed
        return np.degrees(q)

    def plan(self, cube0, place):
        """Return the list of (cartesian_waypoint, gripper_closed, n_steps) phases."""
        c = np.asarray(cube0)
        p = np.array([place[0], place[1]])
        return [
            (np.array([c[0], c[1], 0.16]), False, 30),   # approach above cube
            (np.array([c[0], c[1], 0.083]), False, 26),  # descend to cube
            (np.array([c[0], c[1], 0.083]), True, 20),   # close jaw -> grasp
            (np.array([c[0], c[1], 0.16]), True, 26),    # lift
            (np.array([0.06, -0.18, 0.20]), True, 30),   # high central waypoint
            (np.array([p[0], p[1], 0.16]), True, 40),    # above pad
            (np.array([p[0], p[1], 0.093]), True, 30),   # lower onto pad
            (np.array([p[0], p[1], 0.093]), False, 20),  # open -> release
            (np.array([p[0], p[1], 0.16]), False, 26),   # retract
        ]

    def actions_for_phase(self, cart_target, gripper_closed, nsteps):
        """Yield closed-loop joint-target actions to move the arm to a phase target.

        Re-reads the arm's actual position each step and interpolates toward the IK goal
        (open-loop precomputed interpolation tracks poorly under actuator lag)."""
        q_goal = self.joint_target_deg(cart_target)
        grip = GRIP_CLOSE if gripper_closed else GRIP_OPEN
        for s in range(nsteps):
            a = (s + 1) / nsteps
            q_now = np.degrees(self.e.d.qpos[self.e.arm_q])
            arm = (1 - a) * q_now + a * q_goal
            yield np.concatenate([arm, [grip]]).astype(np.float32)
