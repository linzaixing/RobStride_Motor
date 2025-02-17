# serial_cybergear.py
import time
import motor_protocol as motor_protocol

class ModeRun:
    """电机运行模式控制类"""

    def __init__(self, motor_id: int, port="COM5"):
        """
        初始化电机控制
        :param motor_id: 电机ID
        """
        self.protocol = motor_protocol.CyberGearProtocol(port)
        self.motor_id = motor_id
        self.V_MAX = self.protocol.V_MAX
        self.V_MIN = self.protocol.V_MIN
        self.P_MAX = self.protocol.P_MAX
        
        # 模式状态值
        self.jog_mode_active = False
        self.speed_mode_active = False
        self.iq_mode_active = False
        self.iq_speed_activate = False
        self.jog_speed_cw = 0x2182
        self.jog_speed_ccw = 0xDD7D

    '''JOG模式驱动电机'''
    def jog_control(self, direction: str = "CW", speed_value: int = None) -> bool:
        """
        JOG模式控制电机
        :param direction: 方向，"CW"正转，"CCW"反转
        :param speed_value: 速度值
        :return: 发送是否成功
        """
        if speed_value is None:
            if direction=="CW":
                speed = 0x2182 
            elif direction=="CCW":
                speed = 0xDD7D
        else:
            speed = speed_value

        frame = self.protocol.create_motor_jog_frame(self.motor_id, speed)
        print(f"JOG mode:\t direction:{direction}\t frame: {frame.hex()}")
        return self.protocol.send_data(frame)

    def jog_stop(self) -> bool:
        """停止JOG模式"""
        frame = self.protocol.create_motor_jog_frame_stop(self.motor_id)
        # print(f'JOG mode stop:\t\tframe: {frame.hex()}\n')
        return self.protocol.send_data(frame)
    
    '''模式0：运控模式驱动电机'''
    def motion_control(self) -> None:
        """运控模式控制"""
        delay = 0.1
        frame = self.protocol.create_motor_mode_frame(self.motor_id, self.protocol.index['RUN_MODE'], 0)
        self.protocol.send_data(frame)
         
        frame = self.protocol.create_motor_enable_frame(self.motor_id)
        self.protocol.send_data(frame)
        # TODO
        # 通信类型1
            
    '''模式1：位置模式驱动电机（PP）'''
    def position_control(self, speed_val: float = None, acc_val: float = None, 
                        position_val: float = None) -> None:
        """
        位置模式控制(PP)
        :param speed_val: 速度值
        :param acc_val: 加速度值
        :param position_val: 位置值
        """
        print(f'位置模式PP:\n')
        if speed_val is None:
            speed_val = self.V_MAX  # 使用实例变量作为默认值
        if position_val is None:
            position_val = self.P_MAX 
        if acc_val is None:
            if speed_val >= 0:
                acc_val = 10
            else:
                acc_val = -10
        delay = 0.1
        frame = self.protocol.create_motor_zero_frame(self.motor_id)
        self.protocol.send_data(frame)
        frame = self.protocol.create_motor_mode_frame(self.motor_id, self.protocol.index['RUN_MODE'], 1)
        self.protocol.send_data(frame)

        frame = self.protocol.create_motor_enable_frame(self.motor_id)
        self.protocol.send_data(frame)

        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['vel_max'], speed_val)
        self.protocol.send_data(frame)

        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['acc_set'], acc_val)
        self.protocol.send_data(frame)

        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['LOC_REF'], position_val)
        self.protocol.send_data(frame)
        print(f'spd_ref:{speed_val} acc_set:{acc_val} LOC_REF:{position_val}')
        
    def stop_motion(self) -> None:
        """停止所有运动"""
        frame = self.protocol.create_motor_reset_frame(self.motor_id)
        self.protocol.send_data(frame)
        print(f'电机重置:\tframe: {frame.hex()}\n')
        
    '''模式2：速度模式控制电机'''
    def speed_control(self, iq_val: float = 5, acc_rad: float = 10, 
                     spd_ref: float = 5) -> None:
        """
        速度模式控制
        :param iq_val: 电流值
        :param acc_rad: 加速度
        :param spd_ref: 速度参考值
        """
        print(f'模式2：速度模式控制电机:')
        delay = 0.1
        frame = self.protocol.create_motor_mode_frame(self.motor_id, self.protocol.index['RUN_MODE'], 2)
        self.protocol.send_data(frame)
        frame = self.protocol.create_motor_enable_frame(self.motor_id)
        self.protocol.send_data(frame)
        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['LIMIT_CUR'], abs(iq_val))
        self.protocol.send_data(frame)
        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['acc_rad'], acc_rad)
        self.protocol.send_data(frame)
        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['SPD_REF'], spd_ref)
        print(f'spd_ref:{spd_ref} iq_val: {iq_val} acc_rad: {acc_rad}')
        self.protocol.send_data(frame)
        
    '''模式3：电流模式控制电机'''
    def iq_control(self, iq_val: float = 5) -> None:
        """
        电流模式控制
        :param iq_val: 电流值
        """
        print(f'电流模式控制电机:')
        delay = 0.1
        frame = self.protocol.create_motor_mode_frame(self.motor_id, self.protocol.index['RUN_MODE'], 3)
        self.protocol.send_data(frame)
        frame = self.protocol.create_motor_enable_frame(self.motor_id)
        self.protocol.send_data(frame)
        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['IQ_REF'], iq_val)
        print(f'iq_val: {iq_val}')
        self.protocol.send_data(frame)
        
    '''模式5：位置模式控制电机（CSP）'''
    def position_control_csp(self, speed_val: float = 10, 
                           position_val: float = 6.28) -> None:
        """
        位置模式控制(CSP)
        :param speed_val: 速度值
        :param position_val: 位置值
        """
        print(f'位置模式控制电机（CSP）:')
        delay = 0.1
        frame = self.protocol.create_motor_zero_frame(self.motor_id)
        self.protocol.send_data(frame)
        time.sleep(delay)
        frame = self.protocol.create_motor_mode_frame(self.motor_id, self.protocol.index['RUN_MODE'], 5)
        self.protocol.send_data(frame)
        frame = self.protocol.create_motor_enable_frame(self.motor_id)
        self.protocol.send_data(frame)
        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['LIMIT_SPD'], abs(speed_val))
        self.protocol.send_data(frame)
        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['LOC_REF'], position_val)
        self.protocol.send_data(frame)
        print(f'spd_ref:{speed_val} LOC_REF:{position_val}')
    

    
    
