# EDULITE-A3 ↔ LeRobot 接入

把开源机械臂 **EDULITE-A3**(RobStride,7-DOF,https://github.com/RobStride/EDULITE_A3)
接入 LeRobot,用于遥操作数据采集、策略训练与 VLA 推理。

- `edulite_a3_follower`(本包)—— 执行端 Robot
- `edulite_a3_leader`(`lerobot/teleoperators/edulite_a3_leader`)—— 拖动示教遥操作端

---

## 1. 电机映射

| EDULITE 关节 | CAN id | 电机 | robstride 型号 | 力矩 / 转速 |
|---|---|---|---|---|
| L1 | 1 | RS00 | `O0` | 14 N·m / 33 rad/s |
| L2 | 2 | RS00 | `O0` | 14 N·m / 33 rad/s |
| L3 | 3 | RS00 | `O0` | 14 N·m / 33 rad/s |
| L4 | 4 | EL05 | `ELO5` | 6 N·m / 50 rad/s |
| L5 | 5 | EL05 | `ELO5` | 6 N·m / 50 rad/s |
| L6 | 6 | EL05 | `ELO5` | 6 N·m / 50 rad/s |
| L7(夹爪) | 7 | EL05 | `ELO5` | 6 N·m / 50 rad/s |

关节方向 `[-1,+1,-1,+1,-1,+1,+1]`、软限位(L1 ±160°、L2 0~210°、L3 -230~0°、L4-L7 ±90°)
均镜像自上游 `el_a3_sdk/el_a3_sdk/protocol.py`,在 `get_observation`/`send_action` 里做
逻辑帧↔电机帧换向与裁剪。

---

## 2. 快速使用(真机)

```bash
# 采集示教数据(follower 执行臂 + leader 拖动臂,双 CAN 口)
lerobot-record \
  --robot.type=edulite_a3_follower --robot.port=can0 \
  --robot.cameras='{top: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}' \
  --teleop.type=edulite_a3_leader --teleop.port=can1 \
  --dataset.repo_id=<user>/edulite_a3_pickplace --dataset.num_episodes=50

# 训练(示例:ACT)
lerobot-train --dataset.repo_id=<user>/edulite_a3_pickplace --policy.type=act

# 推理回放
lerobot-eval --policy.path=<ckpt> --robot.type=edulite_a3_follower --robot.port=can0
```

单臂拖动示教(同一条臂既拖又录)时,把 `--teleop.port` 指向同一 `can0` 并用
`use_zero_torque_damping=True`。

---

## 3. ⚠️ 关键风险:CAN 帧格式不一致(必须先处理)

**这是接真机前最重要的一步。** EDULITE 原生栈和 lerobot 内置 robstride 驱动
用的是 RobStride 电机的**两种不同通信协议**,在总线上的字节格式完全不同,
**互不兼容**:

| | **EDULITE 原生**(`el_a3_sdk/can_driver.py`) | **lerobot robstride**(`motors/robstride/robstride.py`) |
|---|---|---|
| 协议 | RobStride **私有协议**(默认) | **MIT 协议** |
| 帧类型 | **扩展帧 29-bit ID**(EFF 标志 `0x80000000`) | **标准帧 11-bit ID**(`is_extended_id=False`) |
| 仲裁 ID | `(comm_type<<24)｜(data_area2<<8)｜motor_id` | `arbitration_id = motor_id` |
| 运控指令 | Type 1:ID 里 `comm_type=1`,**前馈力矩放在 ID 的 bit23~8** | MIT:`arbitration_id=motor_id` |
| 8 字节数据 | `>HHHH` = pos, vel, kp, kd(**大端 4×16-bit**) | MIT 位打包:pos 16b + vel 12b + kp 12b + kd 12b + **torque 12b** |
| 力矩位置 | 在**仲裁 ID** 内 | 在**数据字节**内 |
| 使能帧 | Type 3(ID 编码) | `[0xFF]*7 + [0xFC]`(数据字节) |
| kp/kd 分辨率 | 16-bit(0~500 / 0~5) | 12-bit(0~500 / 0~5) |

> 电机的力矩/转速量程(RS00 14N·m·33rad/s、EL05 6N·m·50rad/s)两边**一致**,
> 差异纯粹在**帧结构与协议**。

### 结论
EDULITE 出厂/默认运行在**私有协议(扩展帧)**;而 lerobot 的 robstride 驱动
只会发**MIT 标准帧**。直接用本适配包连未改协议的 EDULITE 电机,
**握手 / 控制都不会成功**。

### 解决方案(二选一)

**方案 A —— 把电机切到 MIT 协议(改动最小)**
RobStride 电机支持切换协议(私有 ↔ CANopen ↔ MIT,见 `CommType.SET_PROTOCOL=25`
或用官方 MotorStudio)。切到 MIT 标准帧后,本适配包的 `RobstrideMotorsBus` 直接可用。
- 优点:零额外代码
- 缺点:切换后 EDULITE 自带的 SDK/ROS(私有协议)**将无法再驱动这些电机**,
  除非再切回;且需确认 EL05/RS05 固件支持 MIT 标准帧模式。

**方案 B —— 写一个私有协议 MotorsBus(推荐,保真)**
新增一个 `EduliteRobstrideBus(MotorsBusBase)`,复刻 `can_driver.py` 的
`_build_extended_can_id` + `>HHHH` 打包 + `float_to_uint16`,发扩展帧 Type-1 运控指令。
- 优点:与 EDULITE 原生 SDK/ROS **完全一致**,电机无需改协议,可共存
- 缺点:需实现约 300~400 行(可直接移植 `can_driver.py`)

> 本适配包当前基于 lerobot 内置 `RobstrideMotorsBus`(方案 A 前提)。
> 如需方案 B,把 `edulite_a3_follower.py` / `edulite_a3_leader.py` 里的
> `RobstrideMotorsBus` 换成 `EduliteRobstrideBus` 即可(接口保持一致)。

---

## 4. 标定 / 零点

RobStride 电机不在内部存标定。上电流程:
1. 手动把机械臂摆到 home(SDK 零位)姿态
2. `follower.calibrate()` → 调用 `set_zero_position()` 电气置零
3. 零点须与 EDULITE URDF 零位对齐,否则 FK/IK 与数据集坐标会偏

---

## 5. 待真机验证清单

- [ ] **CAN 帧格式**:确认电机协议模式(方案 A 或 B),这是首要阻塞项
- [ ] 帧格式:EDULITE 是经典 CAN 2.0(`use_can_fd=False`),非 CAN FD
- [ ] 零点/方向:与 URDF 零位对齐,验证 7 关节换向符号
- [ ] Kp/Kd 调参:默认 `kp=10, kd=0.5`,按负载调,避免抖动/过冲
- [ ] 控制频率:真机 200Hz,record/inference 常 30-50Hz,确认稳定性
- [ ] 相机:VLA 必需("V"),EDULITE 原仓无视觉,需外接

---

## 6. 文件清单

```
robots/edulite_a3_follower/          执行端 Robot
  config_edulite_a3_follower.py
  edulite_a3_follower.py
teleoperators/edulite_a3_leader/     拖动示教端 Teleoperator
  config_edulite_a3_leader.py
  edulite_a3_leader.py
tests/robots/test_edulite_a3_follower.py         6 用例(mock)
tests/teleoperators/test_edulite_a3_leader.py    6 用例(mock)
```
`robots/utils.py` 与 `teleoperators/utils.py` 各加一个工厂分支用于 CLI 发现。
