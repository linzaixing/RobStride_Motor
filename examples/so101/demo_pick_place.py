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

"""Run the scripted IK expert on the SO-101 pick-and-place env; save two videos.

    python examples/so101/demo_pick_place.py --out /home/opencode/output/videos
"""

import argparse
import os
from pathlib import Path

import numpy as np

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")

import imageio  # noqa: E402

from examples.so101.pick_place_expert import So101IKExpert  # noqa: E402
from lerobot.envs.so101.pick_place_env import So101PickPlaceEnv  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/home/opencode/output/videos")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    env = So101PickPlaceEnv(obs_type="environment_state_agent_pos", randomize_cube=False)
    obs, info = env.reset(seed=args.seed)
    expert = So101IKExpert(env)
    cube0 = info["cube"]

    frames_ext, frames_front = [], []

    def grab():
        frames_ext.append(env.render("cam_ext"))
        frames_front.append(env.render("cam_front"))

    grab()
    last = info
    for cart, gclosed, nsteps in expert.plan(cube0, env.place):
        for action in expert.actions_for_phase(cart, gclosed, nsteps):
            obs, reward, terminated, truncated, last = env.step(action)
            grab()
    for _ in range(30):
        obs, reward, terminated, truncated, last = env.step(
            np.concatenate([np.degrees(env.d.qpos[env.arm_q]), [-60.0]]).astype(np.float32))
        grab()

    print(f"[demo] success={last['is_success']} place_dist={last['place_dist']*1000:.1f}mm "
          f"cube={np.round(last['cube'], 3)}")
    imageio.mimsave(out / "so101_pick_place_external.mp4", frames_ext, fps=30)
    imageio.mimsave(out / "so101_pick_place_frontcam.mp4", frames_front, fps=30)
    print(f"[demo] saved 2 videos to {out}")
    env.close()


if __name__ == "__main__":
    main()
