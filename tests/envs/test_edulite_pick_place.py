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

"""Tests for the EDULITE-A3 pick-and-place env (weld-based grasp, no teleporting)."""

import numpy as np
import pytest

pytest.importorskip("mujoco")

import gymnasium as gym  # noqa: E402
import mujoco  # noqa: E402

from lerobot.envs.configs import (
    EdulitePickPlaceEnv as EdulitePickPlaceEnvConfig,  # noqa: E402
    EnvConfig,  # noqa: E402
)
from lerobot.envs.edulite.pick_place_env import GRIPPER_CLOSE_CMD, EdulitePickPlaceEnv  # noqa: E402


def test_config_registered():
    assert "edulite_pick_place" in EnvConfig.get_known_choices()
    cfg = EdulitePickPlaceEnvConfig()
    assert cfg.gym_id == "gym_edulite/EdulitePickPlace-v0"
    assert cfg.package_name == "lerobot.envs.edulite"
    assert cfg.features["action"].shape == (7,)


@pytest.fixture
def env():
    e = EdulitePickPlaceEnv(
        obs_type="environment_state_agent_pos",
        observation_width=128,
        observation_height=96,
        max_episode_steps=40,
        randomize_ball=False,
    )
    yield e
    e.close()


def test_reset_places_ball_on_floor(env):
    obs, info = env.reset(seed=0)
    assert obs["agent_pos"].shape == (7,)
    assert obs["environment_state"].shape == (9,)
    ball = info["ball"]
    assert abs(ball[2] - 0.028) < 1e-3  # box half-size, resting on the floor


def test_no_teleport_when_gripper_far_from_ball(env):
    """Closing the gripper far from the ball must NOT move/attach the ball."""
    env.reset(seed=0)
    # place ball far from the arm's reach
    env.data.qpos[env._ball_qadr:env._ball_qadr + 3] = [0.4, -0.3, 0.028]
    mujoco.mj_forward(env.model, env.data)
    ball0 = env._ball_xyz()
    closed = np.zeros(7, dtype=np.float32)
    closed[6] = GRIPPER_CLOSE_CMD + 40  # command gripper closed
    for _ in range(10):
        _obs, _r, _term, _trunc, info = env.step(closed)
    assert not info["grasped"]
    # ball did not jump onto the arm
    assert np.linalg.norm(env._ball_xyz()[:2] - ball0[:2]) < 0.02


def test_grasp_engages_on_jaw_contact(env):
    env.reset(seed=0)
    # place the object overlapping a real jaw collision geom so a contact is generated
    env.data.qpos[env._ball_qadr:env._ball_qadr + 3] = env.data.geom_xpos[env._jaw_geoms[0]]
    env.data.qpos[env._ball_qadr + 3:env._ball_qadr + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(env.model, env.data)

    closed = env._agent_pos().copy()
    closed[6] = GRIPPER_CLOSE_CMD + 40  # command gripper closed -> grasp on contact
    grasped = False
    for _ in range(5):
        # keep the object at the jaw each step until contact registers and welds
        if not env._grasped:
            env.data.qpos[env._ball_qadr:env._ball_qadr + 3] = env.data.geom_xpos[env._jaw_geoms[0]]
            mujoco.mj_forward(env.model, env.data)
        _obs, _r, _term, _trunc, info = env.step(closed)
        grasped = grasped or info["grasped"]
        if grasped:
            break
    assert grasped

    # releasing (gripper open) detaches the object
    opened = closed.copy()
    opened[6] = -90.0
    _obs, _r, _term, _trunc, info = env.step(opened)
    assert not info["grasped"]


def test_env_via_gym_make():
    e = gym.make(
        "gym_edulite/EdulitePickPlace-v0",
        obs_type="environment_state_agent_pos",
        max_episode_steps=10,
        disable_env_checker=True,
    )
    obs, _ = e.reset(seed=1)
    assert "agent_pos" in obs
    e.close()
