# SO-101 (SO-ARM100) — LeRobot MuJoCo pick-and-place

Self-contained tabletop pick-and-place simulation for the **SO-101 / SO-ARM100** arm.
Unlike a rigid single-mesh gripper, SO-101 has a **real actuated parallel jaw** with
friction finger pads, so the grasp is a genuine physical friction grasp (a contact-
triggered weld keeps it robust).

- Env: `src/lerobot/envs/so101/pick_place_env.py` → `--env.type=so101_pick_place`
- Bundled model + meshes: `src/lerobot/envs/so101/assets/` (MuJoCo Menagerie, Apache-2.0)

## Interface

```
observation:
  agent_pos          (6,)  5 arm joint angles (deg) + gripper opening
  pixels             (H,W,3) uint8                      [pixels_agent_pos]
  environment_state  (9,)  cube_xyz + target_xyz + pinch_xyz [environment_state_agent_pos]
action:
  Box(6,)  5 arm joint targets (deg) + gripper command (>0 closes)
```

## Run

```bash
pip install -e ".[so101]"
# scripted expert demo (renders 2 videos)
python examples/so101/demo_pick_place.py --out /home/opencode/output/videos
# success-rate benchmark
python examples/so101/pick_place_benchmark.py --episodes 12
# train / eval a policy
lerobot-train --env.type=so101_pick_place --policy.type=act ...
```

## Notes

- Objects sit on a **table**; the arm operates above it, so there is no floor clipping.
- Collision masks: the table collides only with the cube (arm never jams on it); the cube
  rests on the table/pad; the finger pads contact the cube (grasp).
- The scripted expert (`pick_place_expert.py`) uses a 6-DOF-aware MuJoCo IK
  (position + approach-axis-down) with a dual seed and a central carry waypoint;
  scripted success ≈ 75% over random cube poses.
- Model/meshes are from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
  (`trs_so_arm100`), Apache-2.0 — see `assets/LICENSE`.
