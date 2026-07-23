# EDULITE-A3 × LeRobot: Simulation & Sim-to-Real

This directory contains the EDULITE-A3 (7-DOF RobStride arm) MuJoCo simulation task,
scripted expert, and benchmark, plus the recipe for taking a policy from **simulation to
the real robot**.

The whole design rests on one idea: **the simulation env and the real robot expose the
exact same observation / action interface**, so a single policy runs on either one by
swapping `--env.type` for `--robot.type` — no code change.

```
                     ┌──────── one policy (ACT / Diffusion / pi0 / SmolVLA) ────────┐
                     │      observation: { L1.pos … L6.pos , gripper , pixels }      │
                     │      action:      { 6 joint targets (deg) , gripper cmd }     │
                     ▼                                                              ▼
   SIMULATION                                                     REAL ROBOT
   EnvConfig("edulite_pick_place")                                RobotConfig("edulite_a3_follower")
   src/lerobot/envs/edulite/pick_place_env.py                     src/lerobot/robots/edulite_a3_follower/
     MuJoCo physics + offscreen camera                              EduliteA3Follower
     URDF+meshes in envs/edulite/assets/                            └─ EduliteRobstrideBus → CAN → RobStride motors
```

---

## 1. Components (what plays which role)

| Repo component | Role in sim-to-real |
|---|---|
| `envs/edulite/pick_place_env.py` (`--env.type=edulite_pick_place`) | **Sim**: generate data, train, validate |
| `envs/edulite/edulite_env.py` (`--env.type=edulite`) | **Sim**: reach task (simpler) |
| `robots/edulite_a3_follower/` (`--robot.type=edulite_a3_follower`) | **Real execution**: run the policy on hardware |
| `teleoperators/edulite_a3_leader/` (`--teleop.type=edulite_a3_leader`) | **Real data collection**: kinesthetic teleop |
| `motors/robstride` + `edulite_can_bus.py` | **Transport**: CAN driver for the motors |
| `examples/edulite/pick_place_expert.py` | Scripted IK expert (generates sim demos) |
| `examples/edulite/demo_pick_place.py` | Render the task to video |
| `examples/edulite/pick_place_benchmark.py` | Success-rate + failure comparison |

---

## 2. Interface alignment (the reason it transfers)

|  | Sim `EdulitePickPlaceEnv` | Real `EduliteA3Follower` |
|---|---|---|
| `agent_pos` | joint angles, degrees, **logical frame** | joint angles, degrees, logical frame (CAN read-back) |
| `pixels`   | MuJoCo offscreen camera | real camera (OpenCV / RealSense) |
| action     | joint targets (deg, logical) + gripper | same, sent to motors over CAN |

The logical↔motor sign convention (`JOINT_DIRECTIONS`) and joint limits are mirrored from
the upstream EDULITE SDK `el_a3_sdk/el_a3_sdk/protocol.py` on both sides, so the frames match.

---

## 3. Data the transfer depends on

1. **Policy checkpoint + normalization stats** — `model.safetensors` and the dataset
   statistics (mean/std/min/max) used at train time; the real deployment MUST use the
   same stats or the action scale is wrong.
2. **Calibration** — electrical zero (`set_zero_position`), `JOINT_DIRECTIONS`, joint
   limits (already mirrored in `config_edulite_a3_follower.py`).
3. **Camera intrinsics/extrinsics** — the real camera pose/FOV/resolution should match the
   sim camera; keep the same image preprocessing on both sides.
4. **Kinematics/inertia** — `envs/edulite/assets/el_a3.urdf` + meshes (FK verified equal to
   Pinocchio) and `inertia_params.yaml` (mass/COM, used for domain randomization).
5. **Control** — control frequency and MIT gains (`position_kp/kd`); add action latency in
   sim to match the CAN bus.

---

## 4. Workflows

### A. Sim-train → real-deploy
```bash
# 1) train in simulation (GPU recommended)
lerobot-train --env.type=edulite_pick_place --policy.type=act \
  --dataset.repo_id=<user>/edulite_pp_sim

# 2) deploy the SAME checkpoint on hardware (swap env -> robot)
lerobot-eval  --policy.path=outputs/train/.../pretrained_model \
  --robot.type=edulite_a3_follower --robot.port=can0 \
  --robot.cameras='{top:{type:opencv,index_or_path:0,width:640,height:480,fps:30}}'
```

### B. Real teleop-collect → train → deploy (most robust)
```bash
# 1) collect demos by dragging the arm (kinesthetic teaching)
lerobot-record \
  --robot.type=edulite_a3_follower --robot.port=can0 --robot.cameras='{top:{...}}' \
  --teleop.type=edulite_a3_leader --teleop.port=can1 \
  --dataset.repo_id=<user>/edulite_pp_real --dataset.num_episodes=50
# 2) train, 3) eval on the real robot (same commands as above)
```

Sim is then used for pretraining / algorithm validation; real data fine-tunes.

---

## 5. Reducing the reality gap (domain randomization)

Randomize in `EdulitePickPlaceEnv` so the policy is robust to real-world variation:
object pose (already: `randomize_ball`), link mass/friction (base values from
`inertia_params.yaml`), camera extrinsics/lighting/texture, action latency, observation
noise, joint gains.

---

## 6. Real-robot prerequisites (must be handled before deployment)

- [ ] Physical arm + RobStride motors + CAN adapter (Ubuntu, SocketCAN)
- [ ] **CAN protocol mode**: private (`protocol="private"`, `EduliteRobstrideBus`) or MIT
      (`protocol="mit"`, built-in `RobstrideMotorsBus`) — must match the motors' firmware
- [ ] Zero / direction calibration (`calibrate()` at the home pose, aligned to URDF zero)
- [ ] A real camera (the EDULITE repo ships no vision; VLA needs it)
- [ ] Re-tune `position_kp/kd` and control rate under real load

---

## 7. Caveats (honest)

- Pure sim→real zero-shot is hard for contact-rich manipulation (grasping depends on
  contact/vision). Prefer workflow **B** (real demos) or sim pretraining + real fine-tuning.
- The shipped URDF gripper is a single rigid claw; the sim models the grasp with a
  prismatic (linear-slide) DOF + contact-triggered hold, which is a stand-in for the real
  parallel gripper's physics. Fine grasping should be validated/fine-tuned on hardware.
- FK is exact (MuJoCo == Pinocchio == URDF); dynamics/contact are approximate.
