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

"""Run the scripted expert on the EDULITE-A3 pick-and-place env and save two videos.

    python examples/edulite/demo_pick_place.py --out /home/opencode/output/videos

Grasping is done by the environment's weld constraint (no teleporting): the ball stays
on the floor until the gripper closes next to it.
"""

import argparse
import os
from pathlib import Path

import numpy as np

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")

import imageio  # noqa: E402

from examples.edulite.pick_place_expert import PinocchioIK, ScriptedPickPlace  # noqa: E402
from lerobot.envs.edulite.pick_place_env import EdulitePickPlaceEnv  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/home/opencode/output/videos")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    env = EdulitePickPlaceEnv(obs_type="environment_state_agent_pos", randomize_ball=False)
    obs, info = env.reset(seed=args.seed)
    expert = ScriptedPickPlace(PinocchioIK())

    frames_ext, frames_front = [], []

    def grab():
        frames_ext.append(env.render("cam_ext"))
        frames_front.append(env.render("cam_front"))

    grab()
    last = None
    while not expert.done:
        es = obs["environment_state"]
        ball, target, ee = es[0:3], es[3:6], es[6:9]
        action = expert.act(ee, ball, target, jaw=env._jaw_center())
        obs, reward, terminated, truncated, info = env.step(action)
        grab()
        last = info
        if truncated:
            break

    print(f"[demo] grasped={last['grasped']} place_dist={last['place_dist']*1000:.1f}mm "
          f"success={last['is_success']}")
    print(f"[demo] ball final={np.round(last['ball'], 3)} target={np.round(last['target'], 3)}")

    imageio.mimsave(out / "edulite_pick_place_external.mp4", frames_ext, fps=30)
    imageio.mimsave(out / "edulite_pick_place_frontcam.mp4", frames_front, fps=30)
    print(f"[demo] saved 2 videos to {out}")
    env.close()


if __name__ == "__main__":
    main()
