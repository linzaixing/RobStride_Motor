import base_protocol.motor_manager as motor_manager
from base_protocol.serial_manager import SerialConnect
from robot_module.robot_protocol import Robot
import time
import math

def demo_Homing():
    # # 演示demo
    time.sleep(5)
    motor_id=2
    robot_ctr = Robot(port='COM5')
    robot_ctr.CalibrateHomeOffset(motor_id=2, speed_val=0.2)
    robot_ctr.CalibrateHomeOffset(motor_id=3, speed_val=0.2)
    robot_ctr.CalibrateHomeOffset(motor_id=1, speed_val=0.1)
    robot_ctr.CalibrateHomeOffset(motor_id=6, speed_val=1)
    
    robot_ctr.SetRunJoint(motor_id=1, speed=0.51, pos=3.14 / 4)
    robot_ctr.SetRunJoint(motor_id=6, speed=1, pos=3.14 / 2)
    robot_ctr.SetRunJoint(motor_id=2, speed=0.51, pos=3.14 / 2)
    robot_ctr.SetRunJoint(motor_id=3, speed=0.51, pos=3.14 / 2.5)
    time.sleep(7)
    
    robot_ctr.Homing(motor_id=2)
    robot_ctr.Homing(motor_id=1)
    robot_ctr.Homing(motor_id=3)
    robot_ctr.Homing(motor_id=6)
    
    # robot_ctr.SetEnable(motor_id,True)
    
    # robot_ctr.SetTorque(3, tq_val=14)
    print("按x键拖拽示教...\n")
    print("按q键退出程序...\n")
    while True:
        if input() == 'x':
            robot_ctr.SetDisableJoint(motor_id)
            robot_ctr.SetEnable(motor_id,True)
            robot_ctr.SetTorque(motor_id, tq_val=0)
        if input() == 'q':
            robot_ctr.DeinitRobot()
            print("程序结束")
            break
    

if __name__ == "__main__":
    # demo_Homing();
    
    positions = [-0.0219647045875146, -0.01720273585453402, 0.008310587956624767, -0.04210852333746961, 0.01214546902507518, 0.0383061121936588]
    velocities = [-0.12879587499164882, -0.10617575295588436, 0.043764184375565875, -0.2444535451735695, 0.07358888776684064, 0.22839065168094047]

    robot_ctr = Robot(port='COM5')
    
    print('请输入选项')
    while True:
        user_input = input()  # 获取用户输入
        if user_input == 'i': # 必选项，机械臂初始化位姿，自动找零
            # 所有关节找机械零位
            robot_ctr.CalibrateHomeOffset(motor_id=0)          
        if user_input == 'r':  # 连续下发位置速度等信息
            back_pos, back_speed = robot_ctr.ParseFeedbackFata(positions, velocities)
            print(f'back_pos: {back_pos}')
            print(f'back_speed: {back_speed}')
            # robot_ctr.SetTorque(motor_id, tq_val=0)
        elif user_input == 't':  # 示教模式
            # 所有关节电机主动上报位置信息
            robot_ctr.StartReportThread()
        elif user_input == 'q':
            robot_ctr.StopReportThread()
            robot_ctr.DeinitRobot(_backhome=True)
            print("程序结束")
            break
        else:
            time.sleep(0.5)

    






    # """演示函数（带线程支持）"""
    # motor_id = 1
    # mode_run = motor_manager.ModeRun(motor_id)
    
    # # 启动串口监听线程
    # mode_run.start_listening()
    
    # try:
    #     # 示教模式演示
    #     print("\n=== 进入示教模式 ===")
    #     mode_run.start_teach_mode()
    #     input("手动移动电机，按Enter结束示教...")
    #     teach_data = mode_run.stop_teach_mode()
    #     print(f"记录到{len(teach_data)}条示教数据")

    #     time.sleep(1)
        
    #     print("\n=== 复现示教轨迹 ===")
    #     mode_run.replay_teach_mode(teach_data)
    #     time.sleep(1)

    # finally:
    #     # 确保线程关闭
    #     mode_run.stop_listening()
