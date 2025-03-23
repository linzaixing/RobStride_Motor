from base_protocol import motor_manager
import base_protocol.ps5_manager as ps5_manager
import time

def demo() -> None:
    """演示函数"""
    motor_id = 6
    mode_run = motor_manager.ModeRun(motor_id)
    """JOG驱动电机"""
    mode_run.jog_control("CW")
    time.sleep(1)
    mode_run.jog_stop()
    mode_run.jog_control("CCW")
    time.sleep(1)
    mode_run.jog_stop()
    time.sleep(1)
    
    '''模式1：位置模式驱动电机(PP)'''
    mode_run.position_control(speed_val=5, acc_val=10, position_val=3.14)
    time.sleep(3)
    mode_run.disable_motor()
    time.sleep(1)
    
    '''模式2：速度模式驱动电机'''
    mode_run.speed_control()
    time.sleep(2)
    mode_run.disable_motor()
    time.sleep(1)
    
    '''模式3：电流模式驱动电机'''
    mode_run.iq_control(-2)
    time.sleep(0.5)
    mode_run.disable_motor()
    time.sleep(1)
    
    '''模式5：csp位置模式驱动电机'''
    mode_run.position_control_csp(10, 3.14)
    time.sleep(3)
    mode_run.disable_motor()

# 使用示例
if __name__ == "__main__":
    # # 演示demo
    # demo()
    motor_id = 6
    motor_manager.SerialConnect(port="COM5", baudrate=921600)
    ps5_ctr = ps5_manager.ps5_controller(motor_id)
    ps5_ctr.handle_ps5_input()
