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

"""MuJoCo Gymnasium environment for the EDULITE-A3 7-DOF arm (reach task).

Self-contained: the URDF and STL meshes are bundled under ``assets/`` in this
package, so the environment is portable and needs no external robot repo. It is
built from the same URDF as the real robot (FK validated against Pinocchio), and
exposes an interface aligned with :class:`lerobot.robots.edulite_a3_follower`:

    observation:
        "agent_pos" : (7,) joint angles in DEGREES (logical joint frame)
        "pixels"    : (H, W, 3) uint8 RGB camera image (the "V" for VLA)
    action:
        Box(7,) target joint angles in DEGREES (logical joint frame, L1..L7)

Runs headless on CPU via OSMesa. Requires the ``edulite`` extra (mujoco).
"""

from __future__ import annotations

import os
import re
import tempfile
from pathlib import Path

import numpy as np

# Prefer software rendering so the env works on headless / GPU-less machines.
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

JOINTS = ["L1", "L2", "L3", "L4", "L5", "L6", "L7"]
# logical <-> URDF/motor sign convention (mirrors el_a3_sdk protocol / the follower)
JOINT_DIRECTIONS = np.array([-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0])


def _build_scene_mjcf() -> str:
    """URDF -> MJCF (position actuators + camera + offscreen buffer). Returns a path."""
    xml = _URDF.read_text()
    xml = xml.replace("package://el_a3_description/meshes/", "")
    mj = f'<mujoco><compiler meshdir="{_MESHDIR}" balanceinertia="true" discardvisual="false"/></mujoco>'
    xml = re.sub(r"(<robot\b[^>]*>)", r"\1\n  " + mj, xml, count=1)

    tmpdir = Path(tempfile.mkdtemp(prefix="edulite_mjcf_"))
    tmp_urdf = tmpdir / "edulite.urdf"
    tmp_urdf.write_text(xml)

    model = mujoco.MjModel.from_xml_path(str(tmp_urdf))
    mjcf_path = tmpdir / "edulite_scene.xml"
    mujoco.mj_saveLastXML(str(mjcf_path), model)

    mjcf = mjcf_path.read_text()
    jr = {}
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        jr[name] = tuple(model.jnt_range[i])

    acts = ["  <actuator>"]
    for j in JOINTS:
        jn = f"{j}_joint"
        lo, hi = jr.get(jn, (-3.14, 3.14))
        kp = 80.0 if j in ("L1", "L2", "L3") else 20.0
        acts.append(f'    <position name="{jn}_act" joint="{jn}" kp="{kp}" ctrlrange="{lo:.4f} {hi:.4f}"/>')
    acts.append("  </actuator>")

    extras = (
        '  <visual><global offwidth="640" offheight="480"/></visual>\n'
        '  <worldbody>\n'
        '    <light pos="0 0 2" dir="0 0 -1" diffuse="0.9 0.9 0.9"/>\n'
        '    <camera name="cam" pos="0.6 -0.6 0.5" xyaxes="1 1 0 -0.4 0.4 1"/>\n'
        '  </worldbody>\n'
    )
    mjcf = mjcf.replace("</mujoco>", "\n".join(acts) + "\n" + extras + "</mujoco>")
    mjcf_path.write_text(mjcf)
    return str(mjcf_path)


class EduliteReachEnv(gym.Env):
    """EDULITE-A3 reach task: drive the end-effector to a randomized target point."""

    metadata = {"render_modes": ["rgb_array"], "render_fps": 25}

    def __init__(
        self,
        obs_type: str = "pixels_agent_pos",
        render_mode: str = "rgb_array",
        observation_width: int = 640,
        observation_height: int = 480,
        visualization_width: int = 640,
        visualization_height: int = 480,
        max_episode_steps: int = 200,
        success_threshold: float = 0.04,
    ):
        super().__init__()
        if not _MUJOCO_AVAILABLE:
            raise ImportError(
                "mujoco is required for EduliteReachEnv. Install it with: pip install 'lerobot[edulite]'"
            )
        self.obs_type = obs_type
        self.render_mode = render_mode
        self.ow, self.oh = observation_width, observation_height
        self.vw, self.vh = visualization_width, visualization_height
        self.max_episode_steps = max_episode_steps
        self.success_threshold = success_threshold

        self.model = mujoco.MjModel.from_xml_path(_build_scene_mjcf())
        self.data = mujoco.MjData(self.model)
        self._renderer = mujoco.Renderer(self.model, height=self.oh, width=self.ow)

        self._qadr, self._lo_deg, self._hi_deg = [], [], []
        for j in JOINTS:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{j}_joint")
            self._qadr.append(self.model.jnt_qposadr[jid])
            lo, hi = np.degrees(self.model.jnt_range[jid])
            self._lo_deg.append(lo)
            self._hi_deg.append(hi)
        self._qadr = np.array(self._qadr)
        self._lo_deg = np.array(self._lo_deg)
        self._hi_deg = np.array(self._hi_deg)
        self._ee_body = self.model.nbody - 1

        self.action_space = spaces.Box(low=-180.0, high=180.0, shape=(7,), dtype=np.float32)
        obs_spaces = {"agent_pos": spaces.Box(-180.0, 180.0, shape=(7,), dtype=np.float32)}
        if obs_type == "pixels_agent_pos":
            obs_spaces["pixels"] = spaces.Box(0, 255, shape=(self.oh, self.ow, 3), dtype=np.uint8)
        elif obs_type == "environment_state_agent_pos":
            obs_spaces["environment_state"] = spaces.Box(-np.inf, np.inf, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Dict(obs_spaces)

        self._step_count = 0
        self._target = np.array([0.05, -0.12, 0.20], dtype=np.float32)

    # ---- frame conversions ----
    def _agent_pos_deg(self) -> np.ndarray:
        return (np.degrees(self.data.qpos[self._qadr]) * JOINT_DIRECTIONS).astype(np.float32)

    def _action_to_ctrl(self, action) -> np.ndarray:
        motor_deg = np.asarray(action, dtype=float) * JOINT_DIRECTIONS
        motor_deg = np.clip(motor_deg, self._lo_deg, self._hi_deg)
        return np.radians(motor_deg)

    def _get_obs(self) -> dict:
        obs = {"agent_pos": self._agent_pos_deg()}
        if self.obs_type == "pixels_agent_pos":
            self._renderer.update_scene(self.data, camera="cam")
            obs["pixels"] = self._renderer.render()
        elif self.obs_type == "environment_state_agent_pos":
            obs["environment_state"] = self.data.xpos[self._ee_body].astype(np.float32)
        return obs

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self._qadr] = 0.0
        # randomize the reach target within a small workspace box
        self._target = self.np_random.uniform(
            low=[-0.10, -0.18, 0.10], high=[0.10, -0.02, 0.30]
        ).astype(np.float32)
        mujoco.mj_forward(self.model, self.data)
        self._step_count = 0
        return self._get_obs(), {"target": self._target.copy()}

    def step(self, action):
        self.data.ctrl[:] = self._action_to_ctrl(action)
        for _ in range(10):
            mujoco.mj_step(self.model, self.data)
        self._step_count += 1

        ee = self.data.xpos[self._ee_body]
        dist = float(np.linalg.norm(ee - self._target))
        reward = -dist
        success = dist < self.success_threshold
        terminated = bool(success)
        truncated = self._step_count >= self.max_episode_steps
        info = {"is_success": success, "ee_dist": dist, "target": self._target.copy()}
        return self._get_obs(), reward, terminated, truncated, info

    def render(self):
        w, h = (self.vw, self.vh)
        if (h, w) != (self._renderer.height, self._renderer.width):
            self._renderer = mujoco.Renderer(self.model, height=h, width=w)
        self._renderer.update_scene(self.data, camera="cam")
        return self._renderer.render()

    def close(self):
        pass
