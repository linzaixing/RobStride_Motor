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

"""EDULITE-A3 MuJoCo pick-and-place (grasp a ground sphere, place it on a target).

Gripper
-------
The shipped URDF keeps the original ``jaw`` mesh rigidly on the static wrist link and
drives an *empty* ``gripper_link`` with a revolute joint (L7) -- so the gripper can never
move. We keep the ORIGINAL jaw mesh (no substitute geometry) and only fix that
unreasonable DOF: the jaw is attached to ``gripper_link`` and **L7 is changed from a
revolute to a prismatic (linear-slide) joint**, so the real jaw now slides to open/close.

Grasp
-----
The grasp is **contact-based**: the ball is attached (weld) only once the jaw's collision
geometry actually touches the ball while the gripper is commanded closed. Merely being
near the ball does nothing.

Collision
---------
Ball vs {floor, place pad, jaw} and arm-end vs floor are enabled (no ground penetration,
ball rests on the pad without interpenetration). Upper-arm self collisions are disabled
(the URDF neighbours overlap and would otherwise jam the joints).
"""

from __future__ import annotations

import os
import re
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
_URDF = _ASSETS / "el_a3.urdf"
_MESHDIR = _ASSETS / "meshes"

ARM_JOINTS = ["L1", "L2", "L3", "L4", "L5", "L6"]  # 6-DOF arm
ARM_DIRECTIONS = np.array([-1.0, 1.0, -1.0, 1.0, -1.0, 1.0])
GRIPPER_CLOSE_CMD = 30.0  # action[6] >= this => command the gripper (L7 slide) closed
L7_OPEN, L7_CLOSE = 0.005, -0.03  # prismatic jaw slide positions (m): open vs closed
PLACE_XY = (-0.14, -0.05)  # place-target location on the floor


def _build_pick_place_mjcf() -> str:
    xml = _URDF.read_text().replace("package://el_a3_description/meshes/", "")
    mj = f'<mujoco><compiler meshdir="{_MESHDIR}" balanceinertia="true" discardvisual="false"/></mujoco>'
    xml = re.sub(r"(<robot\b[^>]*>)", r"\1\n  " + mj, xml, count=1)
    tmp = Path(tempfile.mkdtemp(prefix="edulite_pp_env_"))
    (tmp / "arm.urdf").write_text(xml)
    model = mujoco.MjModel.from_xml_path(str(tmp / "arm.urdf"))
    mjcf_path = tmp / "scene.xml"
    mujoco.mj_saveLastXML(str(mjcf_path), model)
    mjcf = mjcf_path.read_text()

    # (1) move the two ORIGINAL jaw geoms from the static wrist link onto gripper_link,
    #     shifting z from -0.0755 (wrt wrist) to -0.0015 (wrt gripper_link at -0.074).
    jaw_geoms = re.findall(r'<geom pos="0 0 -0\.0755"[^>]*mesh="jaw"/>', mjcf)
    for g in jaw_geoms:
        mjcf = mjcf.replace(g, "", 1)
    jaw_moved = "".join(g.replace('pos="0 0 -0.0755"', 'pos="0 0 -0.0015"') for g in jaw_geoms)

    # (2) change L7 from revolute to a prismatic (linear-slide) joint and drop the jaw in.
    mjcf = re.sub(
        r'<joint name="L7_joint"[^/]*/>',
        '<joint name="L7_joint" type="slide" pos="0 0 0" axis="0 0 1" '
        'range="-0.03 0.005" damping="2"/>' + jaw_moved,
        mjcf, count=1,
    )

    jr = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i): tuple(model.jnt_range[i])
          for i in range(model.njnt)}
    acts = ["  <actuator>"]
    for j in ARM_JOINTS:
        lo, hi = jr.get(f"{j}_joint", (-3.14, 3.14))
        kp = 400.0 if j in ("L1", "L2", "L3") else 150.0
        kv = 20.0 if j in ("L1", "L2", "L3") else 8.0
        acts.append(f'    <position name="{j}_act" joint="{j}_joint" kp="{kp}" kv="{kv}" '
                    f'ctrlrange="{lo:.4f} {hi:.4f}"/>')
    acts.append('    <position name="L7_act" joint="L7_joint" kp="120" kv="8" ctrlrange="-0.03 0.005"/>')
    acts.append("  </actuator>")

    scene = f"""  <visual><global offwidth="960" offheight="720"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.3 0.3 0.35" rgb2="0.4 0.4 0.45" width="256" height="256"/>
    <material name="grid" texture="grid" texrepeat="6 6" reflectance="0.1"/>
  </asset>
  <worldbody>
    <light pos="0.3 -0.3 1.2" dir="-0.2 0.2 -1" diffuse="1 1 1"/>
    <geom name="floor" type="plane" size="1 1 0.1" material="grid" pos="0 0 0" contype="1" conaffinity="1"/>
    <geom name="place_pad" type="cylinder" size="0.035 0.004" pos="{PLACE_XY[0]} {PLACE_XY[1]} 0.004" rgba="0.15 0.8 0.2 0.9" contype="1" conaffinity="1"/>
    <camera name="cam_ext" pos="0.55 -0.55 0.45" xyaxes="1 1 0 -0.45 0.45 1"/>
    <camera name="cam_front" pos="0.0 -0.7 0.35" xyaxes="1 0 0 0 0.4 1"/>
    <body name="ball" pos="0.05 -0.15 0.028">
      <freejoint name="ball_free"/>
      <geom name="ball_geom" type="box" size="0.028 0.028 0.028" rgba="0.9 0.15 0.15 1" mass="0.05" contype="1" conaffinity="1" friction="1.5 0.05 0.005"/>
    </body>
  </worldbody>
  <contact>
    <exclude body1="l5_l6_urdf_asm" body2="gripper_link"/>
    <exclude body1="ball" body2="l5_l6_urdf_asm"/>
  </contact>
  <equality>
    <weld name="grasp" body1="gripper_link" body2="ball" active="false" solref="0.02 1"/>
  </equality>
"""
    mjcf = mjcf.replace("</mujoco>", "\n".join(acts) + "\n" + scene + "</mujoco>")
    mjcf_path.write_text(mjcf)
    return str(mjcf_path)


class EdulitePickPlaceEnv(gym.Env):
    """Pick a ground sphere and place it on a target pad (original jaw, prismatic DOF)."""

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
        success_threshold: float = 0.05,
        randomize_ball: bool = True,
    ):
        super().__init__()
        if not _MUJOCO_AVAILABLE:
            raise ImportError(
                "mujoco is required for EdulitePickPlaceEnv. Install with: pip install 'lerobot[edulite]'"
            )
        self.obs_type = obs_type
        self.render_mode = render_mode
        self.ow, self.oh = observation_width, observation_height
        self.vw, self.vh = visualization_width, visualization_height
        self.max_episode_steps = max_episode_steps
        self.success_threshold = success_threshold
        self.randomize_ball = randomize_ball

        self.model = mujoco.MjModel.from_xml_path(_build_pick_place_mjcf())
        # Enable collision only on the arm END assembly (wrist + jaw) plus floor/pad/ball;
        # disable it on the upper arm links (whose URDF neighbours overlap and would jam).
        keep_bodies = {"l5_l6_urdf_asm", "gripper_link"}
        self._jaw_geoms = []
        for gid in range(self.model.ngeom):
            gname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid)
            bid = self.model.geom_bodyid[gid]
            bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid)
            mesh_id = self.model.geom_dataid[gid]
            is_mesh_jaw = mesh_id >= 0 and mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_MESH, mesh_id) == "jaw"
            if gname in ("floor", "place_pad", "ball_geom"):
                continue
            if bname in keep_bodies:
                self.model.geom_contype[gid] = 1
                self.model.geom_conaffinity[gid] = 1
                if is_mesh_jaw:
                    self._jaw_geoms.append(gid)
            else:
                self.model.geom_contype[gid] = 0
                self.model.geom_conaffinity[gid] = 0
        self.data = mujoco.MjData(self.model)

        self._rh, self._rw = (self.oh, self.ow) if obs_type == "pixels_agent_pos" else (self.vh, self.vw)
        self._renderer = mujoco.Renderer(self.model, height=self._rh, width=self._rw)

        self._qadr, self._lo_deg, self._hi_deg = [], [], []
        for j in ARM_JOINTS:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{j}_joint")
            self._qadr.append(self.model.jnt_qposadr[jid])
            lo, hi = np.degrees(self.model.jnt_range[jid])
            self._lo_deg.append(lo)
            self._hi_deg.append(hi)
        self._qadr = np.array(self._qadr)
        self._lo_deg = np.array(self._lo_deg)
        self._hi_deg = np.array(self._hi_deg)

        self._arm_act = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{j}_act")
                         for j in ARM_JOINTS]
        self._l7_act = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "L7_act")
        self._l7_qadr = self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "L7_joint")]

        self._ee_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "gripper_link")
        self._ball_jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "ball_free")
        self._ball_qadr = self.model.jnt_qposadr[self._ball_jid]
        self._ball_dofadr = self.model.jnt_dofadr[self._ball_jid]
        self._ball_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "ball")
        self._ball_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "ball_geom")
        self._weld_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "grasp")

        self.action_space = spaces.Box(-180.0, 180.0, shape=(7,), dtype=np.float32)
        obs_spaces = {"agent_pos": spaces.Box(-180.0, 180.0, shape=(7,), dtype=np.float32)}
        if obs_type == "pixels_agent_pos":
            obs_spaces["pixels"] = spaces.Box(0, 255, shape=(self.oh, self.ow, 3), dtype=np.uint8)
        elif obs_type == "environment_state_agent_pos":
            obs_spaces["environment_state"] = spaces.Box(-np.inf, np.inf, shape=(9,), dtype=np.float32)
        self.observation_space = spaces.Dict(obs_spaces)

        self._step_count = 0
        self._grasped = False

    # ---- helpers ----
    def _gripper_opening(self):
        return float((self.data.qpos[self._l7_qadr] - L7_CLOSE) / (L7_OPEN - L7_CLOSE))  # 0 closed..1 open

    def _agent_pos(self):
        arm = np.degrees(self.data.qpos[self._qadr]) * ARM_DIRECTIONS
        return np.concatenate([arm, [self._gripper_opening() * 90.0]]).astype(np.float32)

    def _ee_xyz(self):
        return self.data.xpos[self._ee_body].copy()

    def _ball_xyz(self):
        return self.data.qpos[self._ball_qadr:self._ball_qadr + 3].copy()

    def _target_xyz(self):
        return np.array([PLACE_XY[0], PLACE_XY[1], 0.0])

    def _jaw_touches_ball(self) -> bool:
        for ci in range(self.data.ncon):
            c = self.data.contact[ci]
            g1, g2 = c.geom1, c.geom2
            if (g1 == self._ball_geom and g2 in self._jaw_geoms) or (
                g2 == self._ball_geom and g1 in self._jaw_geoms
            ):
                return True
        return False

    def _jaw_center(self):
        # world center of the jaw collision geometry (the point that must enclose the ball)
        return np.mean([self.data.geom_xpos[g] for g in self._jaw_geoms], axis=0)

    def _jaw_encloses_ball(self, radius=0.045) -> bool:
        return float(np.linalg.norm(self._jaw_center() - self._ball_xyz())) < radius

    def _set_weld(self, active: bool):
        if active:
            b1, b2 = self._ee_body, self._ball_body
            p1, q1 = self.data.xpos[b1], self.data.xquat[b1]
            p2, q2 = self.data.xpos[b2], self.data.xquat[b2]
            neg_q1 = np.zeros(4)
            mujoco.mju_negQuat(neg_q1, q1)
            rel_pos = np.zeros(3)
            mujoco.mju_rotVecQuat(rel_pos, p2 - p1, neg_q1)
            rel_quat = np.zeros(4)
            mujoco.mju_mulQuat(rel_quat, neg_q1, q2)
            ed = self.model.eq_data[self._weld_id]
            ed[0:3] = 0.0
            ed[3:6] = rel_pos
            ed[6:10] = rel_quat
            ed[10] = 1.0
        self.model.eq_active0[self._weld_id] = int(active)
        self.data.eq_active[self._weld_id] = int(active)

    def _get_obs(self):
        obs = {"agent_pos": self._agent_pos()}
        if self.obs_type == "pixels_agent_pos":
            self._renderer.update_scene(self.data, camera="cam_ext")
            obs["pixels"] = self._renderer.render()
        elif self.obs_type == "environment_state_agent_pos":
            obs["environment_state"] = np.concatenate(
                [self._ball_xyz(), self._target_xyz(), self._ee_xyz()]
            ).astype(np.float32)
        return obs

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self._qadr] = 0.0
        self.data.qpos[self._l7_qadr] = L7_OPEN  # start open
        self._set_weld(False)
        self._grasped = False
        self._step_count = 0
        if self.randomize_ball:
            bx = self.np_random.uniform(-0.02, 0.14)
            by = self.np_random.uniform(-0.20, -0.08)
            self.data.qpos[self._ball_qadr:self._ball_qadr + 3] = [bx, by, 0.028]
            self.data.qpos[self._ball_qadr + 3:self._ball_qadr + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(self.model, self.data)
        return self._get_obs(), {"ball": self._ball_xyz(), "target": self._target_xyz()}

    def _apply_action(self, action):
        action = np.asarray(action, dtype=float)
        arm_motor_deg = np.clip(action[:6] * ARM_DIRECTIONS, self._lo_deg, self._hi_deg)
        for i, aid in enumerate(self._arm_act):
            self.data.ctrl[aid] = np.radians(arm_motor_deg[i])
        close_frac = np.clip((action[6] + 90.0) / 180.0, 0.0, 1.0)
        self.data.ctrl[self._l7_act] = L7_OPEN + close_frac * (L7_CLOSE - L7_OPEN)
        return action[6] >= GRIPPER_CLOSE_CMD

    def step(self, action):
        gripper_closed_cmd = self._apply_action(action)
        for _ in range(8):
            mujoco.mj_step(self.model, self.data)
            # contact-based grasp: engage on the first real jaw<->object contact while the
            # gripper is closed (the flat-faced box gives a stable contact, unlike a sphere).
            if not self._grasped and gripper_closed_cmd and self._jaw_touches_ball():
                self.data.qvel[self._ball_dofadr:self._ball_dofadr + 6] = 0.0
                self._set_weld(True)
                mujoco.mj_forward(self.model, self.data)
                self._grasped = True

        ball = self._ball_xyz()
        dist_ee_ball = float(np.linalg.norm(self._jaw_center() - ball))
        if self._grasped and not gripper_closed_cmd:
            self._set_weld(False)
            self.data.qvel[self._ball_dofadr:self._ball_dofadr + 6] = 0.0
            mujoco.mj_forward(self.model, self.data)
            self._grasped = False

        self._step_count += 1
        ball = self._ball_xyz()
        target = self._target_xyz()
        place_dist = float(np.linalg.norm(ball[:2] - target[:2]))
        reward = -dist_ee_ball if not self._grasped else -place_dist + 0.5
        success = (place_dist < self.success_threshold) and (ball[2] < 0.06) and (not self._grasped)
        if success:
            reward += 5.0
        terminated = bool(success)
        truncated = self._step_count >= self.max_episode_steps
        info = {
            "is_success": success,
            "grasped": self._grasped,
            "ee_ball_dist": dist_ee_ball,
            "place_dist": place_dist,
            "ball": ball,
            "target": target,
        }
        return self._get_obs(), reward, terminated, truncated, info

    def render(self, camera: str = "cam_ext"):
        self._renderer.update_scene(self.data, camera=camera)
        return self._renderer.render()

    def close(self):
        pass
