import serial
import logging

class SerialConnect:
    def __init__(self, port="COM5", baudrate=921600):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        # 尝试连接串口
        if not self.check_connection():
            if self.is_port_available():
                self.connect_serial()
            else:
                print(f"端口 {self.port} 已被占用")
                return False
                
    def connect_serial(self):
        """连接串口并检测连接状态"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=921600,
                timeout=0.1
            )
            if self.serial.is_open:
                print(f"成功连接到端口 {self.port}")
                # Enter "AT command mode"
                self.serial.write(bytes.fromhex('41 54 2b 41 54 0d 0a'))
                print("设置AT模式")
                return True
            else:
                print(f"无法打开端口 {self.port}")
                return False
        except serial.SerialException as e:
            print(f"连接端口 {self.port} 失败: {str(e)}")
            return False
        
    def Write(self, data):
        """发送数据"""
        if self.serial and self.serial.is_open:
            return self.serial.write(data)
        else:
            print("串口未打开，无法发送数据")
            return False
    
    def Read(self, timeout=10):
        """接收数据
        :param timeout: 读取超时时间，单位为ms
        """
        timeout_seconds = timeout / 1000.0  # 转换为秒
        if self.serial and self.serial.is_open:
            self.serial.timeout = timeout_seconds  # 设置读取超时时间
            return self.serial.readline()
        else:
            print("串口未打开，无法接收数据")
            return False
        
    def ClearCanData(self, timeout=10):
        """
        清除接收缓冲区中的所有现有消息。

        参数:
        timeout: 等待清除操作的时间（单位：毫秒）。
        """
        timeout_seconds = timeout / 1000.0  # 转换为秒
        self.serial.timeout = timeout_seconds  # 设置读取超时时间
        while True:
            received_msg = self.serial.readline()
            if not received_msg:
                break
            arbitration_id = int.from_bytes(received_msg[2:6], 'big') >> 11 & 0xFF
            # print(
            #     f"Cleared message with ID {arbitration_id}")
        
    def check_connection(self):
        """检查串口连接状态"""
        if self.serial and self.serial.is_open:
            return True
        else:
            print("串口未连接或已断开")
            return False
        
    def is_port_available(self):
        """检查端口是否可用"""
        try:
            test_serial = serial.Serial(port=self.port)
            test_serial.close()
            return True
        except serial.SerialException:
            return False
    
    def disconnect_serial(self):
        """安全断开串口连接"""
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
                print(f"已断开与端口 {self.port} 的连接")
                return True
            except serial.SerialException as e:
                print(f"断开连接时出错: {str(e)}")
                return False
        else:
            print("串口未连接，无需断开")
            return False
        
        