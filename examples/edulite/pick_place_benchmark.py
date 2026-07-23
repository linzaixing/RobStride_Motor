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

"""Benchmark the EDULITE-A3 pick-and-place scripted expert.

Task 1 - success rate over N random ball positions.
Task 2 - success/failure comparison across conditions:
    * normal      : random reachable ball, gripper used normally
    * no_grasp     : the gripper never closes (ablation -> should fail)
    * out_of_reach : the ball is placed far outside the workspace (should fail)

Outputs a success-rate bar chart and a success-vs-failure montage.

    python examples/edulite/pick_place_benchmark.py --episodes 8 --out /home/opencode/output
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
import mujoco  # noqa: E402

from examples.edulite.pick_place_expert import GRIP_OPEN, PinocchioIK, ScriptedPickPlace  # noqa: E402
from lerobot.envs.edulite.pick_place_env import EdulitePickPlaceEnv  # noqa: E402


def run_episode(env, ik, seed, condition="normal", capture=False):
    obs, info = env.reset(seed=seed)
    if condition == "out_of_reach":
        env.data.qpos[env._ball_qadr:env._ball_qadr + 3] = [0.45, -0.35, 0.025]
        env.data.qpos[env._ball_qadr + 3:env._ball_qadr + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(env.model, env.data)
        obs = env._get_obs()
    expert = ScriptedPickPlace(ik)
    frames = []
    info = {"is_success": False, "place_dist": 1.0, "grasped": False, "ball": env._ball_xyz()}
    while not expert.done:
        es = obs["environment_state"]
        ball, target, ee = es[0:3], es[3:6], es[6:9]
        action = expert.act(ee, ball, target, jaw=env._jaw_center())
        if condition == "no_grasp":
            action[6] = GRIP_OPEN  # never close the gripper
        obs, _r, _term, trunc, info = env.step(action)
        if capture:
            frames.append(env.render("cam_ext"))
        if trunc:
            break
    return info, frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=8)
    ap.add_argument("--out", default="/home/opencode/output")
    args = ap.parse_args()
    out = Path(args.out)
    (out / "images").mkdir(parents=True, exist_ok=True)
    (out / "videos").mkdir(parents=True, exist_ok=True)

    env = EdulitePickPlaceEnv(obs_type="environment_state_agent_pos", randomize_ball=True)
    ik = PinocchioIK()

    conditions = ["normal", "no_grasp", "out_of_reach"]
    results = {}
    for cond in conditions:
        succ, dists = [], []
        for e in range(args.episodes):
            info, _ = run_episode(env, ik, seed=100 + e, condition=cond)
            succ.append(bool(info["is_success"]))
            dists.append(float(info["place_dist"]))
        rate = 100.0 * np.mean(succ)
        results[cond] = (rate, float(np.mean(dists)))
        print(f"[bench] {cond:12s}: success {rate:5.1f}%  mean place_dist {np.mean(dists)*1000:6.1f}mm "
              f"({sum(succ)}/{len(succ)})")

    # capture one success (normal) and one failure (out_of_reach) for a comparison montage
    _, ok_frames = run_episode(env, ik, seed=100, condition="normal", capture=True)
    _, bad_frames = run_episode(env, ik, seed=100, condition="out_of_reach", capture=True)
    env.close()

    # --- success-rate bar chart ---
    fig, ax = plt.subplots(figsize=(7, 4.5))
    names = list(results)
    rates = [results[c][0] for c in names]
    colors = ["#2ca02c", "#ff7f0e", "#d62728"]
    ax.bar(names, rates, color=colors)
    for i, r in enumerate(rates):
        ax.text(i, r + 2, f"{r:.0f}%", ha="center", fontweight="bold")
    ax.set_ylim(0, 105)
    ax.set_ylabel("success rate (%)")
    ax.set_title(f"EDULITE-A3 pick-and-place — scripted expert over {args.episodes} episodes/condition")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "images" / "edulite_pick_place_success_rate.png", dpi=130)
    print(f"[bench] saved success-rate chart to {out / 'images'}")

    # --- success vs failure montage ---
    def montage(frames, title, path):
        idx = np.linspace(0, len(frames) - 1, 6).astype(int)
        fig, axs = plt.subplots(1, 6, figsize=(20, 3.6))
        for a, i in zip(axs, idx, strict=False):
            a.imshow(frames[i])
            a.set_title(f"frame {i}", fontsize=9)
            a.axis("off")
        fig.suptitle(title, fontsize=13)
        fig.tight_layout()
        fig.savefig(path, dpi=110)

    if ok_frames and bad_frames:
        montage(ok_frames, "SUCCESS (reachable ball) — grasp + place",
                out / "images" / "edulite_pp_success.png")
        montage(bad_frames, "FAILURE (out-of-reach ball) — no grasp, ball not moved",
                out / "images" / "edulite_pp_failure.png")
        import imageio
        imageio.mimsave(out / "videos" / "edulite_pick_place_failure.mp4", bad_frames, fps=30)
        print("[bench] saved success/failure montages + failure video")


if __name__ == "__main__":
    main()
