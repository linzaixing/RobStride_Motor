# serial_cybergear.py
import time
from . import motor_protocol
from .serial_manager import SerialConnect
import threading
import serial

class ModeRun():
    """电机运行模式控制类"""

    def __init__(self, motor_id: int, serial = None):
        """
        初始化电机控制
        :param motor_id: 电机ID
        """
        self.Serial = serial;
        self.protocol = motor_protocol.CyberGearProtocol()
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
        
        # 数据转换参数
        self.POSITION_RANGE = (-4 * 3.14159, 4 * 3.14159)  # -4π ~ 4π
        self.VELOCITY_RANGE = (-30, 30)                     # -30rad/s ~ 30rad/s
        self.TORQUE_RANGE = (-12, 12)                      # -12Nm ~ 12Nm
        
        # 新增示教模式相关属性
        self.teach_mode_active = False
        self.teach_data = []  # 存储示教数据：[ (position, velocity, timestamp), ... ]
        self.teach_start_time = 0
        
        # 新增串口监听线程相关属性
        self._serial_thread = None
        self._running = False
        self._callback = None
        self.data_buffer = bytearray()
        
        # self.init_motor()
        # self.disable_motor()
        self.Serial.ClearCanData()
        self.enable_motor()
        
    def init_motor(self):
        """初始化所有电机"""
        delay = 0.01

        test_sequence = [
            (delay, "Motor Enable", self.protocol.create_motor_enable_frame(self.motor_id)),
            (delay, "Set Zero Position", self.protocol.create_motor_zero_frame(self.motor_id)),
            (delay, "Set Position Mode", self.protocol.create_motor_mode_frame(self.motor_id, self.protocol.index['RUN_MODE'], 1)),
            (delay, "Set Speed Limit", self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['LIMIT_SPD'], 2.0)),
            (delay, "Set acc Limit", self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['acc_set'], 10)),
            (delay, "Set Initial Position", self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['LOC_REF'], 0))
        ]
        
        for delay, desc, frame in test_sequence:
            print(f"\nSending {desc} frame: {frame.hex()}")
            self.send_data(frame)
            time.sleep(delay)

    def send_data(self, data):
        """发送数据"""
        try:
            # self.Serial.ClearCanData()
            self.Serial.Write(data)
            time.sleep(0.01)
            return True
        except Exception as e:
            print(f"发送数据失败: {e}")
            return False 
            
    def enable_motor(self):
        '''使能电机'''
        self.Serial.ClearCanData()
        frame = self.protocol.create_motor_enable_frame(self.motor_id)
        print(f"\n使能电机: {self.motor_id}")
        self.send_data(frame)
        
    def set_zeropoint(self):
        '''设置电机零位'''
        # self.Serial.ClearCanData()
        frame = self.protocol.create_motor_zero_frame(self.motor_id)
        self.send_data(frame)

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
        return self.send_data(frame)

    def jog_stop(self) -> bool:
        """停止JOG模式"""
        frame = self.protocol.create_motor_jog_frame_stop(self.motor_id)
        # print(f'JOG mode stop:\t\tframe: {frame.hex()}\n')
        return self.send_data(frame)
    
    '''模式0：运控模式驱动电机'''
    def motion_control(self) -> None:
        """运控模式控制"""
        delay = 0.1
        frame = self.protocol.create_motor_mode_frame(self.motor_id, self.protocol.index['RUN_MODE'], 0)
        self.send_data(frame)
         
        frame = self.protocol.create_motor_enable_frame(self.motor_id)
        self.send_data(frame)
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
        # self.Serial.ClearCanData()
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
        # frame = self.protocol.create_motor_zero_frame(self.motor_id)
        # self.send_data(frame)
        frame = self.protocol.create_motor_mode_frame(self.motor_id, self.protocol.index['RUN_MODE'], 1)
        self.send_data(frame)

        # frame = self.protocol.create_motor_enable_frame(self.motor_id)
        # self.send_data(frame)

        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['vel_max'], speed_val)
        self.send_data(frame)

        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['acc_set'], acc_val)
        self.send_data(frame)

        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['LOC_REF'], position_val)
        self.send_data(frame)
        print(f'motor_id: {self.motor_id}, spd_ref:{speed_val} acc_set:{acc_val} LOC_REF:{position_val}')
        
    def disable_motor(self) -> None:
        """电机失能"""
        # self.Serial.ClearCanData()
        frame = self.protocol.create_motor_reset_frame(self.motor_id)
        self.send_data(frame)
        print(f'电机{self.motor_id}失能:\tframe: {frame.hex()}\n')
        
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
        self.send_data(frame)
        # frame = self.protocol.create_motor_enable_frame(self.motor_id)
        # self.send_data(frame)
        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['LIMIT_CUR'], abs(iq_val))
        self.send_data(frame)
        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['acc_rad'], acc_rad)
        self.send_data(frame)
        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['SPD_REF'], spd_ref)
        print(f'motor_id: {self.motor_id}, spd_ref:{spd_ref} iq_val: {iq_val} acc_rad: {acc_rad}')
        self.send_data(frame)
        
    '''模式3：电流模式控制电机'''
    def iq_control(self, iq_val: float = 5) -> None:
        """
        电流模式控制
        :param iq_val: 电流值
        """
        print(f'电流模式控制电机:')
        delay = 0.1
        frame = self.protocol.create_motor_mode_frame(self.motor_id, self.protocol.index['RUN_MODE'], 3)
        self.send_data(frame)
        frame = self.protocol.create_motor_enable_frame(self.motor_id)
        self.send_data(frame)
        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['IQ_REF'], iq_val)
        self.send_data(frame)
        print(f'motor_id: {self.motor_id}, iq_val: {iq_val}')
        
    '''模式5：位置模式控制电机（CSP）'''
    def position_control_csp(self, speed_val: float = 10, 
                           position_val: float = 6.28) -> None:
        """
        位置模式控制(CSP)
        :param speed_val: 速度值
        :param position_val: 位置值
        """
        print(f'位置模式控制电机（CSP）:')
        frame = self.protocol.create_motor_mode_frame(self.motor_id, self.protocol.index['RUN_MODE'], 5)
        self.send_data(frame)
        frame = self.protocol.create_motor_enable_frame(self.motor_id)
        self.send_data(frame)
        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['LIMIT_SPD'], abs(speed_val))
        self.send_data(frame)
        frame = self.protocol.create_motor_write_frame(self.motor_id, self.protocol.index['LOC_REF'], position_val)
        self.send_data(frame)
        print(f'motor_id: {self.motor_id}, spd_ref:{speed_val} LOC_REF:{position_val}')
        
        # 接收电机返回的数据
        data = self.Serial.Read(timeout = 10)
        result = self.protocol.parse_frame(data)
        if not result:
            # print('接收帧为空')
            return None, None
        
        if result['mode'] == 0x02:  # 反馈模式
            signal = self.data_parsing_from_8byte(result['payload'])
        else:
            signal = None
        return result, signal

    def start_listening(self, update_callback=None):
        """启动串口监听线程"""
        self._running = True
        self._callback = update_callback
        self._serial_thread = threading.Thread(
            target=self._serial_read_loop, 
            daemon=True
        )
        self._serial_thread.start()
        print("串口监听线程已启动")

    def stop_listening(self):
        """停止串口监听线程"""
        self._running = False
        if self._serial_thread and self._serial_thread.is_alive():
            self._serial_thread.join(timeout=1)
        print("串口监听线程已停止")

    def _process_raw_data(self, data):
        """处理原始字节数据"""
        self.data_buffer.extend(data)
        
        # 使用协议头定位完整帧
        while True:
            start_idx = self.data_buffer.find(b'AT')
            if start_idx == -1:
                break
                
            # 检查帧完整性
            end_idx = self.data_buffer.find(b'\r\n', start_idx)
            if end_idx == -1:
                break
                
            # 提取完整帧
            frame = self.data_buffer[start_idx:end_idx+2]
            del self.data_buffer[:end_idx+2]
            
            # 调用process处理完整帧
            self.process(frame, self._callback)

    def _scale_value(self, value, in_min, in_max, out_min, out_max):
        """线性映射值的范围"""
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def test_communication(self):
        """
        测试串口通信是否正常。
        """
        try:
            # 发送测试消息
            test_message = [0x41, 0x54, 0x90, 0x07, 0xE8, 0x0C, 0x08, 0x05, 0x70, 0x00, 0x00, 0x07, 0x01, 0x95, 0x54, 0x0D, 0x0A]
            response = self.send_and_receive_signal(data1=test_message)
            
            if response is not None:
                print("Serial communication test successful")
                return True
            else:
                print("Serial communication test failed")
                return False
        except Exception as e:
            print(f"Serial communication test error: {str(e)}")
            return False
    
    def send_and_receive_signal(self, motor_id):
        # self.Serial.ClearCanData()
        frame = self.protocol.create_motor_write_frame( 
            motor_id,
            self.protocol.index['vel_max'],
            10
        )
        self.send_data(frame)
        
        # time.sleep(0.01)
        
        # 接收电机返回的数据
        data = self.Serial.Read(timeout = 10)
        result = self.protocol.parse_frame(data)
        if not result:
            # print('接收帧为空')
            return None, None
        
        if result['mode'] == 0x02 and result['motor_id'] == motor_id:  # 反馈模式
            signal = self.data_parsing_from_8byte(result['payload'])
            # if signal:
            #     print(f"motor_id: {result['motor_id']}, "
            #         f"current_position: {signal['current_position']:>7.3f}, "
            #         f"current_velocity: {signal['current_velocity']:>7.3f}, "
            #         f"current_torque: {signal['current_torque']:>7.3f}, "
            #         f"current_temperature: {signal['current_temperature']:>7.3f}")
        else:
            signal = None
        
        return result, signal
    
    def ReceiveSignal(self):
        # 接收电机返回的数据
        data = self.Serial.Read(timeout = 10)
        if not data:  # 增加数据有效性检查
            print('未接收到数据')
            return None, None
            
        result = self.protocol.parse_frame(data)
        if not result:
            print('接收帧为空')
            return None, None
        
        signal = self.data_parsing_from_8byte(result['payload'])
        return result, signal
    
    def data_parsing_from_8byte(self, data_bytes):
        """从反馈数据更新电机状态
        Args:
            data_bytes: 8字节的反馈数据
        """
        if len(data_bytes) != 8:
            raise ValueError("Feedback data must be 8 bytes")
            
        # 解析位置数据 (Byte 0-1)
        position_raw = int.from_bytes(data_bytes[0:2], 'big', signed=False)
        self.current_position = self.protocol.uint_to_float(position_raw, self.protocol.P_MIN, self.protocol.P_MAX, 16)
        
        # 解析速度数据 (Byte 2-3)
        velocity_raw = int.from_bytes(data_bytes[2:4], 'big', signed=False)
        self.current_velocity = self.protocol.uint_to_float(velocity_raw, self.protocol.V_MIN, self.protocol.V_MAX, 16)
        
        # 解析力矩数据 (Byte 4-5)
        torque_raw = int.from_bytes(data_bytes[4:6], 'big', signed=False)
        self.current_torque = self.protocol.uint_to_float(torque_raw, self.protocol.T_MIN, self.protocol.T_MAX, 16)
        
        # 解析温度数据 (Byte 6-7)
        self.current_temperature = int.from_bytes(data_bytes[6:8], 'big', signed=False)  / 10
        
        return {
            'current_position': self.current_position,
            'current_velocity': self.current_velocity,
            'current_torque': self.current_torque,
            'current_temperature': self.current_temperature
        }
    
    def start_teach_mode(self):
        """进入示教模式并开始记录"""
        print("启动示教模式，开始记录运动数据...")
        # 新增初始化设置
        self._init_teach_mode()
        self.teach_mode_active = True
        self.teach_data.clear()
        self.teach_start_time = time.time()
        
    def stop_teach_mode(self):
        """停止示教模式并返回记录数据"""
        self.teach_mode_active = False
        self.disable_motor()
        print("示教模式已停止")
        return self.teach_data.copy()
    
    def _init_teach_mode(self):
        """示教模式初始化设置"""
        # 1. 设置反馈模式
        frame = self.protocol.create_motor_mode_frame(
            self.motor_id, 
            self.protocol.index['RUN_MODE'], 
            0x02  # 反馈模式
        )
        self.send_data(frame)
        
        # # 设置零位
        # frame = self.protocol.create_motor_zero_frame(self.motor_id)
        # self.send_data(frame)
        
        # # 2. 使能电机
        # frame = self.protocol.create_motor_enable_frame(self.motor_id)
        # self.send_data(frame)
        
        # 3. reset电机
        frame = self.protocol.create_motor_reset_frame(self.motor_id)
        self.send_data(frame)
        
        # 4. 设置反馈频率（示例：100ms）
        frame = self.protocol.create_motor_write_frame(
            self.motor_id,
            self.protocol.index['CUR_FILT_GAIN'],  # 使用滤波器增益索引示例
            100    # 反馈间隔(ms)
        )
        self.send_data(frame)
        time.sleep(0.1)
    
    def replay_teach_mode(self, teach_data):
        """复现示教轨迹"""
        if not teach_data:
            print("无示教数据可复现")
            return
            
        print(f"开始复现{len(teach_data)}条轨迹...")
        start_time = time.time()
        prev_time = teach_data[0][2]
        
        for data in teach_data:
            # 计算时间差并等待
            elapsed = data[2] - prev_time
            time.sleep(max(elapsed, 0))
            
            # 使用位置模式复现
            self.position_control(
                speed_val=abs(data[1])*2,  # 2倍速度复现
                position_val=data[0],
                acc_val=20
            )
            prev_time = data[2]
            
            # 实时显示进度
            progress = (time.time() - start_time) / (teach_data[-1][2] + 0.1) * 100
            print(f"复现进度: {min(progress, 100):.1f}%", end='\r')
            
        self.disable_motor()
        print("\n轨迹复现完成")

    def _process_impl(self, data, update_callback):
        """处理接收到的数据（增强反馈验证）"""
        result = self.protocol.parse_frame(data)
        if not result:
            return None
            
        if result['mode'] == 0x02:  # 反馈模式
            motor_id = result['motor_id']
            payload = result['payload']
            
            # 验证数据长度
            if len(payload) != 8:
                print(f"无效反馈数据长度：{len(payload)}字节")
                return
                
            # 解析电机位置状态
            self.data_parsing_from_8byte(payload)
            
            # 示教模式记录数据（增加有效性检查）
            if self.teach_mode_active and motor_id == self.motor_id:
                if all([
                    self.current_position is not None,
                    self.current_velocity is not None,
                    abs(self.current_velocity) < self.V_MAX  # 过滤异常值
                ]):
                    timestamp = time.time() - self.teach_start_time
                    self.teach_data.append((
                        round(self.current_position, 4),
                        round(self.current_velocity, 4),
                        round(timestamp, 3)
                    ))
                    
    def SetmotorTorque(self, motor_id: int, tq_val=7):
        # self.Serial.ClearCanData()
        frame = self.protocol.create_motor_mode_frame(motor_id, self.protocol.index['RUN_MODE'], 1)
        self.send_data(frame)
        frame = self.protocol.create_motor_write_frame( 
            motor_id,
            self.protocol.index['IMIT_TORQUE'],
            tq_val
        )
        self.send_data(frame)
        # frame = self.protocol.create_motor_control_frame(1,
        #                                          tq_val,
        #                                          1,
        #                                          -0.2,
        #                                          0,
        #                                          0)
        # self.send_data(frame)
        
        