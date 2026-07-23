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

"""SO-101 (SO-ARM100) MuJoCo pick-and-place task for LeRobot.

Self-contained: the SO-ARM100 MuJoCo model (from MuJoCo Menagerie, Apache-2.0) and its
meshes are bundled under ``assets/``. Unlike the EDULITE arm, SO-101 has a REAL actuated
parallel jaw with friction finger pads, so the grasp is a genuine physical friction
grasp (a contact-triggered weld keeps it robust).

Objects sit on a table (the arm operates above it — no floor clipping). Collision masks
give: cube<->table/floor/pad and finger-pad<->cube, while the arm never jams on the table.

Interface (aligned with the SO-101 follower's joint space):
    observation:
        "agent_pos"         : (6,) = 5 arm joint angles (deg) + gripper opening
        "pixels"            : (H,W,3) uint8                        [pixels_agent_pos]
        "environment_state" : (9,) cube_xyz + target_xyz + pinch_xyz [environment_state_agent_pos]
    action:
        Box(6,) = 5 arm joint targets (deg) + gripper command (>0 closes)
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

import numpy as np

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")

import gymnasium as gym  # noqa: E402
from gymnasium import spaces  # noqa: E402

try:
    import mujoco  # noqa: E402

    _MUJOCO_AVAILABLE = True
except ImportError:
    _MUJOCO_AVAILABLE = False

_ASSETS = Path(__file__).parent / "assets"
_ARM_XML = _ASSETS / "so_arm100.xml"

ARM = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"]
JAW_OPEN, JAW_CLOSE = 0.9, -0.15          # Jaw actuator ctrl (open wide vs press closed)
GRIPPER_CLOSE_CMD = 30.0                  # action[5] >= this => command the gripper closed
READY_SEED = [0.0, -1.4, 1.4, -1.2, 0.0]  # bent "ready" arm pose (rad)
PLACE_XY = (0.15, -0.11)


def _build_scene_mjcf() -> str:
    """A pick-place scene that includes the bundled SO-ARM100 model + table/cube/pad."""
    scene = f"""<mujoco model="so101 pick_place">
  <include file="{_ARM_XML}"/>
  <visual><global offwidth="960" offheight="720"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.3 0.3 0.35" rgb2="0.4 0.4 0.45" width="256" height="256"/>
    <material name="grid" texture="grid" texrepeat="6 6" reflectance="0.1"/>
    <material name="wood" rgba="0.72 0.55 0.35 1"/>
  </asset>
  <worldbody>
    <light pos="0.3 -0.3 1.2" dir="-0.2 0.2 -1" diffuse="1 1 1"/>
    <geom name="floor" type="plane" size="1 1 0.05" material="grid"/>
    <camera name="cam_ext" pos="0.45 -0.5 0.45" xyaxes="1 1 0 -0.4 0.4 1"/>
    <camera name="cam_front" pos="0.0 -0.62 0.34" xyaxes="1 0 0 0 0.4 1"/>
    <geom name="table" type="box" size="0.17 0.14 0.025" pos="0.06 -0.18 0.025" material="wood"/>
    <body name="cube" pos="0.0 -0.20 0.068">
      <freejoint name="cube_free"/>
      <geom name="cube_geom" type="box" size="0.018 0.018 0.018" rgba="0.9 0.15 0.15 1" mass="0.03" friction="2.0 0.1 0.01"/>
    </body>
    <geom name="place_pad" type="cylinder" size="0.03 0.004" pos="{PLACE_XY[0]} {PLACE_XY[1]} 0.054" rgba="0.15 0.8 0.2 0.9"/>
  </worldbody>
  <equality>
    <weld name="grasp" body1="Fixed_Jaw" body2="cube" active="false" solref="0.02 1"/>
  </equality>
</mujoco>
"""
    path = Path(tempfile.mkdtemp(prefix="so101_pp_")) / "scene.xml"
    path.write_text(scene)
    return str(path)


class So101PickPlaceEnv(gym.Env):
    """SO-101 (SO-ARM100) pick a table cube and place it on a target pad."""

    metadata = {"render_modes": ["rgb_array"], "render_fps": 30}

    def __init__(
        self,
        obs_type: str = "pixels_agent_pos",
        render_mode: str = "rgb_array",
        observation_width: int = 640,
        observation_height: int = 480,
        visualization_width: int = 960,
        visualization_height: int = 720,
        max_episode_steps: int = 300,
        randomize_cube: bool = True,
    ):
        super().__init__()
        if not _MUJOCO_AVAILABLE:
            raise ImportError(
                "mujoco is required for So101PickPlaceEnv. Install with: pip install 'lerobot[so101]'"
            )
        self.obs_type = obs_type
        self.render_mode = render_mode
        self.ow, self.oh = observation_width, observation_height
        self.vw, self.vh = visualization_width, visualization_height
        self.max_episode_steps = max_episode_steps
        self.randomize_cube = randomize_cube

        self.m = mujoco.MjModel.from_xml_path(_build_scene_mjcf())
        # Stiffen the arm actuators (the Menagerie ±3.5 N·m servos droop under reaching
        # poses in this task) and set collision masks so the table collides only with the
        # cube (arm never jams on it).
        for n in ARM:
            a = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
            self.m.actuator_gainprm[a, 0] = 250.0
            self.m.actuator_biasprm[a, 1] = -250.0
            self.m.actuator_biasprm[a, 2] = -20.0
            self.m.actuator_forcerange[a] = [-50.0, 50.0]
        tg = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, "table")
        cg = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
        self.m.geom_contype[tg] = 4
        self.m.geom_conaffinity[tg] = 4
        self.m.geom_contype[cg] = 5
        self.m.geom_conaffinity[cg] = 5
        self.d = mujoco.MjData(self.m)

        self._rh, self._rw = (self.oh, self.ow) if obs_type == "pixels_agent_pos" else (self.vh, self.vw)
        self._renderer = mujoco.Renderer(self.m, height=self._rh, width=self._rw)

        def jid(n):
            return mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)

        def aid(n):
            return mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)

        self.arm_q = np.array([self.m.jnt_qposadr[jid(n)] for n in ARM])
        self.arm_range = np.array([self.m.jnt_range[jid(n)] for n in ARM])
        self.arm_act = [aid(n) for n in ARM]
        self.jaw_act = aid("Jaw")
        self.jaw_q = self.m.jnt_qposadr[jid("Jaw")]
        self.jaw_range = self.m.jnt_range[jid("Jaw")]
        self.sid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, "pinch")
        self.cube_q = self.m.jnt_qposadr[jid("cube_free")]
        self.cube_dof = self.m.jnt_dofadr[jid("cube_free")]
        self.cube_geom = cg
        self.cube_body = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "cube")
        self.fj_body = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "Fixed_Jaw")
        self.weld = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_EQUALITY, "grasp")
        self.pad_geoms = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, n)
                          for n in ("fixed_jaw_pad_1", "fixed_jaw_pad_2", "fixed_jaw_pad_3",
                                    "fixed_jaw_pad_4", "moving_jaw_pad_1", "moving_jaw_pad_2",
                                    "moving_jaw_pad_3", "moving_jaw_pad_4")]
        self.place = np.array(PLACE_XY)

        self.action_space = spaces.Box(-180.0, 180.0, shape=(6,), dtype=np.float32)
        obs_spaces = {"agent_pos": spaces.Box(-180.0, 180.0, shape=(6,), dtype=np.float32)}
        if obs_type == "pixels_agent_pos":
            obs_spaces["pixels"] = spaces.Box(0, 255, shape=(self.oh, self.ow, 3), dtype=np.uint8)
        elif obs_type == "environment_state_agent_pos":
            obs_spaces["environment_state"] = spaces.Box(-np.inf, np.inf, shape=(9,), dtype=np.float32)
        self.observation_space = spaces.Dict(obs_spaces)

        self._step_count = 0
        self._grasped = False

    # ---- helpers ----
    def _gripper_opening(self):
        lo, hi = self.jaw_range
        return float((self.d.qpos[self.jaw_q] - lo) / (hi - lo))

    def _agent_pos(self):
        arm = np.degrees(self.d.qpos[self.arm_q])
        return np.concatenate([arm, [self._gripper_opening() * 90.0]]).astype(np.float32)

    def pinch_xyz(self):
        return self.d.site_xpos[self.sid].copy()

    def cube_xyz(self):
        return self.d.qpos[self.cube_q:self.cube_q + 3].copy()

    def _target_xyz(self):
        return np.array([self.place[0], self.place[1], 0.058])

    def _pad_touches_cube(self):
        for ci in range(self.d.ncon):
            c = self.d.contact[ci]
            if (c.geom1 == self.cube_geom and c.geom2 in self.pad_geoms) or (
                c.geom2 == self.cube_geom and c.geom1 in self.pad_geoms
            ):
                return True
        return False

    def _set_weld(self, active):
        if active:
            b1, b2 = self.fj_body, self.cube_body
            p1, q1 = self.d.xpos[b1], self.d.xquat[b1]
            p2, q2 = self.d.xpos[b2], self.d.xquat[b2]
            nq1 = np.zeros(4)
            mujoco.mju_negQuat(nq1, q1)
            rp = np.zeros(3)
            mujoco.mju_rotVecQuat(rp, p2 - p1, nq1)
            rq = np.zeros(4)
            mujoco.mju_mulQuat(rq, nq1, q2)
            ed = self.m.eq_data[self.weld]
            ed[0:3] = 0.0
            ed[3:6] = rp
            ed[6:10] = rq
            ed[10] = 1.0
        self.m.eq_active0[self.weld] = int(active)
        self.d.eq_active[self.weld] = int(active)

    def _update_grasp(self, closing):
        if not self._grasped and closing and self._pad_touches_cube():
            self.d.qvel[self.cube_dof:self.cube_dof + 6] = 0.0
            self._set_weld(True)
            self._grasped = True
        elif self._grasped and not closing:
            self._set_weld(False)
            self.d.qvel[self.cube_dof:self.cube_dof + 6] = 0.0
            self._grasped = False

    def _get_obs(self):
        obs = {"agent_pos": self._agent_pos()}
        if self.obs_type == "pixels_agent_pos":
            self._renderer.update_scene(self.d, camera="cam_ext")
            obs["pixels"] = self._renderer.render()
        elif self.obs_type == "environment_state_agent_pos":
            obs["environment_state"] = np.concatenate(
                [self.cube_xyz(), self._target_xyz(), self.pinch_xyz()]
            ).astype(np.float32)
        return obs

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.m, self.d)
        self._set_weld(False)
        self._grasped = False
        self._step_count = 0
        for i, v in enumerate(READY_SEED):
            self.d.qpos[self.arm_q[i]] = v
        self.d.qpos[self.jaw_q] = JAW_OPEN
        for i, act in enumerate(self.arm_act):
            self.d.ctrl[act] = self.d.qpos[self.arm_q[i]]
        self.d.ctrl[self.jaw_act] = JAW_OPEN
        if self.randomize_cube:
            cx = self.np_random.uniform(-0.05, 0.09)
            cy = self.np_random.uniform(-0.23, -0.15)
        else:
            cx, cy = 0.0, -0.20
        self.d.qpos[self.cube_q:self.cube_q + 3] = [cx, cy, 0.068]
        self.d.qpos[self.cube_q + 3:self.cube_q + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(self.m, self.d)
        return self._get_obs(), {"cube": self.cube_xyz(), "target": self._target_xyz()}

    def _apply_action(self, action):
        action = np.asarray(action, dtype=float)
        arm_rad = np.clip(np.radians(action[:5]), self.arm_range[:, 0], self.arm_range[:, 1])
        for i, act in enumerate(self.arm_act):
            self.d.ctrl[act] = arm_rad[i]
        # gripper: action[5] in [-90(open)..+90(close)] -> Jaw ctrl
        close_frac = np.clip((action[5] + 90.0) / 180.0, 0.0, 1.0)
        self.d.ctrl[self.jaw_act] = JAW_OPEN + close_frac * (JAW_CLOSE - JAW_OPEN)
        return action[5] >= GRIPPER_CLOSE_CMD

    def step(self, action):
        closing = self._apply_action(action)
        for _ in range(6):
            mujoco.mj_step(self.m, self.d)
            self._update_grasp(closing)
        self._step_count += 1

        cube = self.cube_xyz()
        target = self._target_xyz()
        pinch = self.pinch_xyz()
        place_dist = float(np.linalg.norm(cube[:2] - target[:2]))
        reward = -float(np.linalg.norm(pinch - cube)) if not self._grasped else -place_dist + 0.5
        success = place_dist < 0.05 and 0.06 < cube[2] < 0.10 and not self._grasped
        if success:
            reward += 5.0
        terminated = bool(success)
        truncated = self._step_count >= self.max_episode_steps
        info = {"is_success": success, "grasped": self._grasped, "place_dist": place_dist,
                "cube": cube, "target": target}
        return self._get_obs(), reward, terminated, truncated, info

    def render(self, camera: str = "cam_ext"):
        self._renderer.update_scene(self.d, camera=camera)
        return self._renderer.render()

    def close(self):
        pass
