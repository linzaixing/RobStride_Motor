import pybullet as p
import pybullet_data
import math
import time

# 连接物理引擎并设置重力
p.connect(p.GUI)
# p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 加载机械臂URDF（示例使用KUKA IIWA，可能需要调整路径）
robot = p.loadURDF(r".\docs\arm_07_urdf\urdf\arm_07_urdf.urdf", [0, 0, 0], useFixedBase=True)

# 获取可动关节索引（过滤固定关节）
joint_indices = [i for i in range(p.getNumJoints(robot)) if p.getJointInfo(robot, i)[2] != p.JOINT_FIXED]
print('joint_indices: ', len(joint_indices))
assert len(joint_indices) >= 6, "机械臂需要至少6个可动关节"
controlled_joints = joint_indices[:6]  # 取前6个可动关节

# 获取末端执行器链接索引（最后一个控制关节的子链接）
last_joint_index = controlled_joints[-1]
joint_info = p.getJointInfo(robot, last_joint_index)
end_effector_link_index = joint_info[16]  # 使用jointInfo的childLinkIndex字段

# 保存初始关节位置
initial_joint_positions = [p.getJointState(robot, i)[0] for i in controlled_joints]

# 初始化目标位置和姿态（基于当前末端状态）
link_state = p.getLinkState(robot, end_effector_link_index)
assert link_state is not None, "无法获取末端执行器状态，请检查链接索引"
current_end_pos, current_end_orn = link_state[:2]
target_pos = list(current_end_pos)
target_rpy = list(p.getEulerFromQuaternion(current_end_orn))

# 主控制循环
print("控制说明:")
print("l/r: X轴增减1cm")
print("f/b: Y轴增减1cm")
print("u/d: Z轴增减1cm")
print("1/2: Roll增减1度")
print("3/4: Pitch增减1度")
print("5/6: Yaw增减1度")
print("Q/q: 退出并复位")

running = True
step_size = 0.01  # 1cm
angle_step = math.radians(1)  # 1度转弧度

while running:
    keys = p.getKeyboardEvents()
    
    # 处理按键事件
    for k, v in keys.items():
        if v & p.KEY_WAS_TRIGGERED:
            if k == ord('q'):
                running = False
            
            # 位置控制
            elif k == ord('l'):
                target_pos[0] += step_size
            elif k == ord('r'):
                target_pos[0] -= step_size
            elif k == ord('f'):
                target_pos[1] += step_size
            elif k == ord('b'):
                target_pos[1] -= step_size
            elif k == ord('u'):
                target_pos[2] += step_size
            elif k == ord('d'):
                target_pos[2] -= step_size
            
            # 姿态控制
            elif k == ord('1'):
                target_rpy[0] += angle_step
            elif k == ord('2'):
                target_rpy[0] -= angle_step
            elif k == ord('3'):
                target_rpy[1] += angle_step
            elif k == ord('4'):
                target_rpy[1] -= angle_step
            elif k == ord('5'):
                target_rpy[2] += angle_step
            elif k == ord('6'):
                target_rpy[2] -= angle_step
                
            else:
                pass

    # 计算逆运动学
    target_quat = p.getQuaternionFromEuler(target_rpy)
    joint_angles = p.calculateInverseKinematics(
        robot,
        end_effector_link_index,
        target_pos,
        target_quat,
        maxNumIterations=100,
        residualThreshold=1e-5,
        jointDamping=[0.1] * 6
        # jointIndices=controlled_joints
    )

    # 应用关节控制
    for i, joint_index in enumerate(controlled_joints):
        p.setJointMotorControl2(
            robot,
            joint_index,
            p.POSITION_CONTROL,
            targetPosition=joint_angles[i],
            force=500
        )
    
    # 步进仿真
    p.stepSimulation()
    time.sleep(1./240.)

# 退出时复位关节位置
for i, pos in zip(controlled_joints, initial_joint_positions):
    p.resetJointState(robot, i, pos)
    p.setJointMotorControl2(
        robot,
        i,
        p.POSITION_CONTROL,
        targetPosition=pos,
        force=500
    )

# 最后步进一次确保复位
p.stepSimulation()
time.sleep(0.5)
p.disconnect()