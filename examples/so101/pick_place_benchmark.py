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

"""Success-rate benchmark for the SO-101 pick-and-place scripted expert.

    python examples/so101/pick_place_benchmark.py --episodes 12 --out /home/opencode/output
"""

import argparse
import os
from pathlib import Path

import numpy as np

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from examples.so101.pick_place_expert import So101IKExpert  # noqa: E402
from lerobot.envs.so101.pick_place_env import So101PickPlaceEnv  # noqa: E402


def run_episode(env, seed):
    obs, info = env.reset(seed=seed)
    expert = So101IKExpert(env)
    last = info
    for cart, gclosed, nsteps in expert.plan(info["cube"], env.place):
        for action in expert.actions_for_phase(cart, gclosed, nsteps):
            obs, _r, _t, _tr, last = env.step(action)
    for _ in range(30):
        obs, _r, _t, _tr, last = env.step(
            np.concatenate([np.degrees(env.d.qpos[env.arm_q]), [-60.0]]).astype(np.float32))
    return last


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=12)
    ap.add_argument("--out", default="/home/opencode/output")
    args = ap.parse_args()
    out = Path(args.out)
    (out / "images").mkdir(parents=True, exist_ok=True)

    env = So101PickPlaceEnv(obs_type="environment_state_agent_pos", randomize_cube=True)
    succ, dists = [], []
    for e in range(args.episodes):
        info = run_episode(env, seed=100 + e)
        succ.append(bool(info["is_success"]))
        dists.append(float(info["place_dist"]))
    env.close()
    rate = 100.0 * np.mean(succ)
    print(f"[bench] SO-101 pick-place success {rate:.1f}%  mean place_dist "
          f"{np.mean(dists) * 1000:.1f}mm  ({sum(succ)}/{len(succ)})")

    fig, ax = plt.subplots(figsize=(5, 4.5))
    ax.bar(["SO-101\n(SO-ARM100)"], [rate], color="#2ca02c")
    ax.text(0, rate + 2, f"{rate:.0f}%", ha="center", fontweight="bold")
    ax.set_ylim(0, 105)
    ax.set_ylabel("success rate (%)")
    ax.set_title(f"SO-101 tabletop pick-and-place\nreal parallel-jaw grasp, {args.episodes} episodes")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "images" / "so101_pick_place_success_rate.png", dpi=130)
    print(f"[bench] saved chart to {out / 'images'}")


if __name__ == "__main__":
    main()
