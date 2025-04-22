import pybullet as p
import pybullet_data
import math
import time

class PybulletRobot():
    def __init__(self):
        # 连接物理引擎并设置重力
        p.connect(p.GUI)
        # p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # 加载机械臂URDF
        self.robotId = p.loadURDF(r".\docs\arm_07_urdf\urdf\arm_07_urdf.urdf", [0, 0, 0], useFixedBase=True)

        # 获取可动关节索引（过滤固定关节）
        joint_indices = [i for i in range(p.getNumJoints(self.robotId)) if p.getJointInfo(self.robotId, i)[2] != p.JOINT_FIXED]
        print('joint_indices: ', len(joint_indices))
        assert len(joint_indices) >= 6, "机械臂需要至少6个可动关节"
        self.controlled_joints = joint_indices[:6]  # 取前6个可动关节

        # 获取末端执行器链接索引（最后一个控制关节的子链接）
        self.last_joint_index = self.controlled_joints[-1]
        joint_info = p.getJointInfo(self.robotId, self.last_joint_index)
        end_effector_link_index = joint_info[16]  # 使用jointInfo的childLinkIndex字段
        # print(joint_indices,"\n", controlled_joints, "\n", last_joint_index, "\n", joint_info, "\n",end_effector_link_index)

        # 保存初始关节位置
        self.initial_joint_positions = [p.getJointState(self.robotId, i)[0] for i in self.controlled_joints]

        # 初始化目标位置和姿态（基于当前末端状态）
        link_state = p.getLinkState(self.robotId, end_effector_link_index)
        assert link_state is not None, "无法获取末端执行器状态，请检查链接索引"
        current_end_pos, current_end_orn = link_state[:2]
        self.target_pos = list(current_end_pos)
        self.target_rpy = list(p.getEulerFromQuaternion(current_end_orn))
        
    def cal_inv_run(self):
        # 计算逆运动学
        target_quat = p.getQuaternionFromEuler(self.target_rpy)
        joint_angles = p.calculateInverseKinematics(
            self.robotId,
            self.last_joint_index,
            self.target_pos,
            target_quat,
            maxNumIterations=100,
            residualThreshold=1e-5,
            jointDamping=[0.1] * 6
            # jointIndices=controlled_joints
        )
        print('\n move target_pos: ', self.target_pos)
        print('\n move target_rpy: ', self.target_rpy)
        print('joint_angles:', joint_angles)

        # 应用关节控制
        for i, joint_index in enumerate(self.controlled_joints):
            p.setJointMotorControl2(
                self.robotId,
                joint_index,
                p.POSITION_CONTROL,
                targetPosition=joint_angles[i],
                force=500
            )
        
        # 步进仿真
        p.stepSimulation()
        time.sleep(1./240.)
        
    def deinit(self):
        # 退出时复位关节位置
        for i, pos in zip(self.controlled_joints, self.initial_joint_positions):
            p.resetJointState(self.robotId, i, pos)
            p.setJointMotorControl2(
                self.robotId,
                i,
                p.POSITION_CONTROL,
                targetPosition=pos,
                force=500
            )

        # 最后步进一次确保复位
        p.stepSimulation()
        time.sleep(0.5)
        p.disconnect()

if __name__ == "__main__":
    py_robot = PybulletRobot()
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
                    py_robot.target_pos[0] += step_size
                elif k == ord('r'):
                    py_robot.target_pos[0] -= step_size
                elif k == ord('f'):
                    py_robot.target_pos[1] += step_size
                elif k == ord('b'):
                    py_robot.target_pos[1] -= step_size
                elif k == ord('u'):
                    py_robot.target_pos[2] += step_size
                elif k == ord('d'):
                    py_robot.target_pos[2] -= step_size
                
                # 姿态控制
                elif k == ord('1'):
                    py_robot.target_rpy[0] += angle_step
                elif k == ord('2'):
                    py_robot.target_rpy[0] -= angle_step
                elif k == ord('3'):
                    py_robot.target_rpy[1] += angle_step
                elif k == ord('4'):
                    py_robot.target_rpy[1] -= angle_step
                elif k == ord('5'):
                    py_robot.target_rpy[2] += angle_step
                elif k == ord('6'):
                    py_robot.target_rpy[2] -= angle_step
                    
                else:
                    pass

        py_robot.cal_inv_run()
        
    py_robot.deinit()