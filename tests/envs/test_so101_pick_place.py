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

"""Tests for the self-contained SO-101 (SO-ARM100) MuJoCo pick-and-place env."""

import numpy as np
import pytest

pytest.importorskip("mujoco")

import gymnasium as gym  # noqa: E402
import mujoco  # noqa: E402

from lerobot.envs.configs import EnvConfig, So101PickPlaceEnv as So101Config
from lerobot.envs.so101.pick_place_env import GRIPPER_CLOSE_CMD, So101PickPlaceEnv


def test_config_registered():
    assert "so101_pick_place" in EnvConfig.get_known_choices()
    cfg = So101Config()
    assert cfg.gym_id == "gym_so101/So101PickPlace-v0"
    assert cfg.package_name == "lerobot.envs.so101"
    assert cfg.features["action"].shape == (6,)
    assert cfg.features["agent_pos"].shape == (6,)


def test_config_creates_env():
    cfg = So101Config(episode_length=15, observation_width=128, observation_height=96)
    envs = cfg.create_envs(n_envs=1, use_async_envs=False)
    vec = next(iter(next(iter(envs.values())).values()))
    obs, _ = vec.reset(seed=0)
    assert obs["agent_pos"].shape == (1, 6)
    assert obs["pixels"].shape == (1, 96, 128, 3)
    vec.close()


@pytest.fixture
def env():
    e = So101PickPlaceEnv(
        obs_type="environment_state_agent_pos",
        max_episode_steps=20,
        randomize_cube=False,
    )
    yield e
    e.close()


def test_reset_places_cube_on_table(env):
    obs, info = env.reset(seed=0)
    assert obs["agent_pos"].shape == (6,)
    assert obs["environment_state"].shape == (9,)
    assert abs(info["cube"][2] - 0.068) < 1e-3  # on the table top


def test_grasp_engages_on_pad_contact(env):
    env.reset(seed=0)
    # move the cube between the finger pads so a pad contact is generated, then close
    env.data = env.d  # alias
    pad0 = env.d.geom_xpos[env.pad_geoms[1]]
    env.d.qpos[env.cube_q:env.cube_q + 3] = pad0
    env.d.qpos[env.cube_q + 3:env.cube_q + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(env.m, env.d)
    closed = env._agent_pos().copy()
    closed[5] = GRIPPER_CLOSE_CMD + 60  # command gripper closed
    grasped = False
    for _ in range(6):
        if not env._grasped:
            env.d.qpos[env.cube_q:env.cube_q + 3] = env.d.geom_xpos[env.pad_geoms[1]]
            mujoco.mj_forward(env.m, env.d)
        _obs, _r, _t, _tr, info = env.step(closed)
        grasped = grasped or info["grasped"]
        if grasped:
            break
    assert grasped
    opened = closed.copy()
    opened[5] = -90.0
    _obs, _r, _t, _tr, info = env.step(opened)
    assert not info["grasped"]


def test_env_via_gym_make():
    e = gym.make(
        "gym_so101/So101PickPlace-v0",
        obs_type="environment_state_agent_pos",
        max_episode_steps=10,
        disable_env_checker=True,
    )
    obs, _ = e.reset(seed=1)
    assert "agent_pos" in obs
    a = np.zeros(6, dtype=np.float32)
    obs, r, term, trunc, info = e.step(a)
    assert "is_success" in info
    e.close()
