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

"""Self-contained SO-101 (SO-ARM100) MuJoCo pick-and-place env for LeRobot.

Importing this package registers the gym environment so that ``EnvConfig``
(type="so101_pick_place") can create it via ``gym.make``.

The bundled SO-ARM100 model and meshes come from MuJoCo Menagerie (Apache-2.0);
see ``assets/LICENSE``.
"""

from gymnasium.envs.registration import register

register(
    id="gym_so101/So101PickPlace-v0",
    entry_point="lerobot.envs.so101.pick_place_env:So101PickPlaceEnv",
    max_episode_steps=300,
)
