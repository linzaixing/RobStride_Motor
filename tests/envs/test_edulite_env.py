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

"""Tests for the self-contained EDULITE-A3 MuJoCo sim env (reach task)."""

import numpy as np
import pytest

pytest.importorskip("mujoco")

import gymnasium as gym  # noqa: E402

from lerobot.envs.configs import EduliteEnv, EnvConfig  # noqa: E402


def test_env_config_registered():
    assert "edulite" in EnvConfig.get_known_choices()
    cfg = EduliteEnv()
    assert cfg.gym_id == "gym_edulite/EduliteReach-v0"
    assert cfg.package_name == "lerobot.envs.edulite"
    # feature shapes align with the real EduliteA3Follower (7-DOF)
    assert cfg.features["action"].shape == (7,)
    assert cfg.features["agent_pos"].shape == (7,)
    assert "pixels" in cfg.features


def test_config_creates_env():
    """EnvConfig.create_envs should import the package, register and build the env."""
    cfg = EduliteEnv(episode_length=20, observation_width=128, observation_height=96)
    envs = cfg.create_envs(n_envs=1, use_async_envs=False)
    # {suite: {task_id: VectorEnv}}
    vec = next(iter(next(iter(envs.values())).values()))
    obs, _ = vec.reset(seed=0)
    assert "agent_pos" in obs and "pixels" in obs
    assert obs["agent_pos"].shape == (1, 7)
    vec.close()


@pytest.fixture
def env():
    import lerobot.envs.edulite  # noqa: F401  (registers the gym id)

    e = gym.make(
        "gym_edulite/EduliteReach-v0",
        obs_type="pixels_agent_pos",
        observation_width=128,
        observation_height=96,
        max_episode_steps=15,
        disable_env_checker=True,
    )
    yield e
    e.close()


def test_spaces_and_reset(env):
    assert env.action_space.shape == (7,)
    obs, info = env.reset(seed=1)
    assert obs["agent_pos"].shape == (7,)
    assert obs["pixels"].shape == (96, 128, 3)
    assert obs["pixels"].dtype == np.uint8
    assert "target" in info


def test_step_moves_and_rewards(env):
    env.reset(seed=2)
    target = np.array([30, 50, -60, 25, -20, 15, 10], dtype=np.float32)
    last = None
    for _ in range(15):
        obs, reward, terminated, truncated, info = env.step(target)
        last = info
        assert reward <= 0.0  # negative distance
        if terminated or truncated:
            break
    # the arm actually moved away from the all-zero start pose
    assert np.abs(obs["agent_pos"]).sum() > 1.0
    assert "ee_dist" in last


def test_action_limits_are_clipped(env):
    env.reset(seed=3)
    # command absurd targets; env must clip internally and not crash / explode
    obs, *_ = env.step(np.full(7, 9999.0, dtype=np.float32))
    assert np.all(np.isfinite(obs["agent_pos"]))
