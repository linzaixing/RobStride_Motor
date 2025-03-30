import struct

class CyberGearProtocol:
    """CyberGear电机通信协议类"""
    def __init__(self):
        # 基本协议配置
        self.AT_HEADER = 'AT'.encode()
        self.END_BYTES = '\r\n'.encode()
        self.master_id = int('0x00fd', 16)
        
        # 参数范围常量
        self.T_MIN, self.T_MAX = -14, 14.0      # 力矩范围
        self.P_MIN, self.P_MAX = -12.56, 12.56      # 位置范围（-4pi~4pi）
        self.V_MIN, self.V_MAX = -33.0, 33.0      # 速度范围
        self.KP_MIN, self.KP_MAX = 0.0, 500.0     # 位置环增益范围
        self.KD_MIN, self.KD_MAX = 0.0, 5.0       # 速度环增益范围
        self.I_MIN, self.I_MAX = 0, 16.0      # 电流范围

        # 电机参数索引
        self.index = {
            'RUN_MODE': 0x7005,        # 运行模式
            'IQ_REF': 0x7006,          # 电流模式Iq指令     -16~16A
            'SPD_REF': 0x700A,         # 转速模式转速指令   -33~33rad/s
            'IMIT_TORQUE': 0x700B,     # 力矩限制           0~14Nm
            'CUR_KP': 0x7010,          # 电流环比例增益
            'CUR_KI': 0x7011,          # 电流环积分增益
            'CUR_FILT_GAIN': 0x7014,   # 电流滤波器增益
            'LOC_REF': 0x7016,         # 位置模式角度指令
            'LIMIT_SPD': 0x7017,       # 位置模式（CSP）速度限制 0~33rad/s
            'LIMIT_CUR': 0x7018,       # 速度位置模式电流限制    0~16A
            'vel_max': 0x7024,         # 位置模式（PP）速度     默认值10rad/s
            'acc_rad': 0x7022,         # 速度模式加速度
            'acc_set': 0x7025          # 位置模式（PP）加速度   默认值10rad/s^2
        }

    def create_frame(self, mode, motor_id, res, data, payload=None):
        """创建通信帧
        Args:
            mode: 命令模式
            motor_id: 电机ID
            res: 保留字段
            data: 数据字段
            payload: 负载数据
        Returns:
            bytearray: 完整的通信帧
        """
        if mode == 7:
            can_id = (res << 29) | (mode << 24) | data | (motor_id & 0xFF)
        else:
            can_id = (res << 29) | (mode << 24) | (data << 8) | (motor_id & 0xFF)
        can_id = (can_id << 3) + 0x04 # 如果需要使用usb转can模块，需要进行转换
        frame = bytearray()
        frame.extend(self.AT_HEADER)
        frame.extend(can_id.to_bytes(4, 'big'))
        if payload:
            frame.append(len(payload))
            frame.extend(payload)
        else:
            frame.append(0)
        frame.extend(self.END_BYTES)
        # print(f'motor_id: {motor_id}, create_frame: {frame.hex()}')
        return frame

    def cat_motor_McuId(self, motor_id):
        '''获取设备的ID和64位MCU唯一标识符(模式 0x0)'''
        payload = bytearray(8)  # 默认初始化为全0
        print(f'获取设备{motor_id}的ID和64位MCU唯一标识符')
        return self.create_frame(0, motor_id, 0, self.master_id, payload)
    
    def create_motor_control_frame(self, motor_id, torque, position, speed, kp, kd):
        """创建电机控制帧 (运控模式 0x1)
        Args:
            motor_id: 电机ID
            torque: 力矩值
            position: 目标位置
            speed: 目标速度
            kp: 位置环增益
            kd: 速度环增益
        """
        data = self.float_to_uint(torque, self.T_MIN, self.T_MAX, 16)
        payload = bytearray(8)
        pos_uint = self.float_to_uint(position, self.P_MIN, self.P_MAX, 16)
        spd_uint = self.float_to_uint(speed, self.V_MIN, self.V_MAX, 16)
        kp_uint = self.float_to_uint(kp, self.KP_MIN, self.KP_MAX, 16)
        kd_uint = self.float_to_uint(kd, self.KD_MIN, self.KD_MAX, 16)
        
        # 左闭右开区间
        payload[0:2] = pos_uint.to_bytes(2, 'little')
        payload[2:4] = spd_uint.to_bytes(2, 'little')
        payload[4:6] = kp_uint.to_bytes(2, 'little')
        payload[6:8] = kd_uint.to_bytes(2, 'little')
        
        return self.create_frame(1, motor_id, 0, data, payload)

    def create_motor_enable_frame(self, motor_id):
        """创建电机使能帧 (模式 0x3)"""
        return self.create_frame(3, motor_id, 0, self.master_id)

    def create_motor_reset_frame(self, motor_id):
        """创建电机复位帧 (模式 0x4)"""
        return self.create_frame(4, motor_id, 0, self.master_id)

    def create_motor_zero_frame(self, motor_id):
        """创建电机零位设置帧 (模式 0x6)"""
        payload = bytearray([0] * 8)
        payload[0] = 1
        print(f"电机{motor_id}设置零点")
        return self.create_frame(6, motor_id, 0, self.master_id, payload)
    
    def reset_motor_canid(self, motor_id, reset_can_id):
        '''更改当前电机CAN_ID , 立即生效（模式 0x7）'''
        # 将主CAN_ID放在bit15~8
        data = (self.master_id & 0xFF) << 8
        # 将预设置CAN_ID放在bit16~23
        data |= (reset_can_id & 0xFF) << 16
        print(f'电机ID{motor_id}重置为ID{reset_can_id}')
        return self.create_frame(7, motor_id, 0, data)
    
    def create_motor_jog_frame(self, motor_id, jog_speed):
        """创建电机jog模式设置帧 (模式 0x12)"""
        index = self.index['RUN_MODE']    # write
        run_mode = 0x07 # jog mode
        payload = bytearray(8)
        payload[0:2] = index.to_bytes(2, 'little')
        payload[4] = run_mode
        payload[5] = 0x01 # 1: enable jog, 0: disable jog
        payload[6:8] = jog_speed.to_bytes(2, 'little')
        return self.create_frame(0x12, motor_id, 0, self.master_id, payload)
    
    def create_motor_jog_frame_stop(self, motor_id):
        """创建电机jog模式设置帧 (模式 0x12)"""
        index = self.index['RUN_MODE']    # write
        run_mode = 0x07 # jog mode
        jog_speed = 0xff7f # zero speed
        payload = bytearray(8)
        payload[0:2] = index.to_bytes(2, 'little')
        payload[4] = run_mode
        payload[5] = 0x00 # 1: enable jog, 0: disable jog
        payload[6:8] = jog_speed.to_bytes(2, 'little')
        return self.create_frame(0x12, motor_id, 0, self.master_id, payload)

    def create_motor_read_frame(self, motor_id, index):
        """创建电机参数读取帧 (模式 0x11)"""
        payload = bytearray(8)
        payload[0:2] = index.to_bytes(2, 'little')
        return self.create_frame(0x11, motor_id, 0, self.master_id, payload)
    
    def create_motor_mode_frame(self, motor_id, index, run_mode):
        """创建电机模式设置帧 (模式 0x12)"""
        payload = bytearray(8)
        payload[0:2] = index.to_bytes(2, 'little')
        payload[4] = run_mode
        return self.create_frame(0x12, motor_id, 0, self.master_id, payload)

    def create_motor_write_frame(self, motor_id, index, value):
        """
        创建电机参数写入帧 (模式 0x12)
        单个参数写入（掉电丢失）
        """
        payload = bytearray(8)
        payload[0:2] = index.to_bytes(2, 'little')
        payload[4:8] = struct.pack('<f', value)  # 小端格式打包浮点数
        return self.create_frame(0x12, motor_id, 0, self.master_id, payload)

    def create_motor_report_frame(self, motor_id, enable_report=False):
        """创建电机主动上报帧 (模式 0x18)
        Args:
            motor_id: 目标电机ID
            enable_report: 是否开启主动上报
        Returns:
            bytearray: 完整的通信帧
        """
        # 命令模式为0x18
        mode = 0x18
        # 主ID放在bit23-8，数据区Byte0为开关，其余补零
        if enable_report:
            print(f'电机{motor_id}主动上报')
        else:
            print(f'电机{motor_id}结束上报')
        payload = bytearray([0x01] + [0x02] + [0x03] + [0x04] + [0x05] + [0x06] + [0x01 if enable_report else 0x00])
        return self.create_frame(24, motor_id, 0, self.master_id, payload)

    def float_to_uint(self, x, x_min, x_max, bits):
        """浮点数转换为无符号整数
        bits = 16，则映射区间为 [0, 65535]
        Args:
            x: 输入浮点数
            x_min: 最小值
            x_max: 最大值
            bits: 位数
        Returns:
            uint: 转换后的无符号整数
        """
        span = x_max - x_min
        offset = x_min
        x = min(max(x, x_min), x_max)
        return int((x - offset) * ((1 << bits) - 1) / span)
    
    def uint_to_float(self, x, x_min, x_max, bits):
        """
        将无符号整数转换为浮点数。

        参数:
        x: 输入的无符号整数。
        x_min: 可接受的最小浮点数。
        x_max: 可接受的最大浮点数。
        bits: 输入无符号整数的位数。

        返回:
        转换后的浮点数。
        """
        span = (1 << bits) - 1
        offset = x_max - x_min
        x = max(min(x, span), 0)  # Clamp x to the range [0, span]
        return offset * x / span + x_min

    def parse_frame(self, data):
        """解析通信帧
        Args:
            data: 接收到的数据
        Returns:
            dict: 解析后的数据字典
        """
        if not data or len(data) < 7:
            return None
        
        if data[0:2] != self.AT_HEADER or data[-2:] != self.END_BYTES:
            return None
        
        can_id = int.from_bytes(data[2:6], 'big')
        # 如果使用usb转can模块，需要进行转换
        can_id = can_id >> 3   # 右移3位
        # print(f"can_id: {hex(can_id)}")

        data_length = data[6]
        payload = data[7:7+data_length]
        
        return {
            'res': (can_id >> 29) & 0xFF,
            'mode': (can_id >> 24) & 0xFF,
            'motor_id': (can_id >> 8) & 0xFF,
            'data': can_id & 0xFF,
            'payload': payload
        }