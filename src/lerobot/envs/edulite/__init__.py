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

"""Self-contained EDULITE-A3 MuJoCo simulation env for LeRobot.

Importing this package registers the gym environment, so that
``EnvConfig`` (type="edulite") can create it via ``gym.make``.
"""

from gymnasium.envs.registration import register

register(
    id="gym_edulite/EduliteReach-v0",
    entry_point="lerobot.envs.edulite.edulite_env:EduliteReachEnv",
    max_episode_steps=200,
)

register(
    id="gym_edulite/EdulitePickPlace-v0",
    entry_point="lerobot.envs.edulite.pick_place_env:EdulitePickPlaceEnv",
    max_episode_steps=300,
)
