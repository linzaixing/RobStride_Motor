from base_protocol.motor_manager import ModeRun
from base_protocol.serial_manager import SerialConnect
import time
import logging
import math
import threading
from . import robot_data  # 使用相对导入

class Robot():
    def __init__(self, port='COM5') -> None:
        self.joint_nums = 6
        try:
            self.Serial = SerialConnect(port)
        except Exception as e:
            print(f"端口 {port} 被占用或无法打开: {str(e)}")
            exit(1)
        self.motorJ = {}
        self.isEnabled = False
        self.find_zeropoints_flag = False
        
        self.init_joint_angle = [0,0,0,0,0,0]

        self.default_joint_speed = [0,0,0,0,0,0]
        self.default_joint_current = [0,0,0,0,0,0]
        self.default_joint_angle = [0,0,0,0,0,0]
        self.default_joint_acc = [0,0,0,0,0,0]
        
        self.joints_zero_dirction = [1,1,-1,-1,1,1]   # 电机归零方向
        self.joints_run_dirction = [-1,-1,1,1,-1,-1]   # 电机运动方向
        self.joints_zero_point = [0,0,0,0,0,0]  # 关节零位
        self.joints_toque_point = [7,7,7,7,7,7]  # 关节扭矩
        
        self.rd_buffer = robot_data.JointDataBuffer(self.joint_nums)
        
        self.report_flag = False
        
        self.SetEnable(0, True)
    
    '''
    关节电机使能
    @param motor_id: motor_id<=0 所有关节使能； motor_id>0 当前关节使能
    @param _enable: 使能（True）
    '''
    def SetEnable(self, motor_id: int, _enable=False, _setzeropoint = False):
        if _enable:
            if motor_id <= 0:
                # 所有关节使能
                for i in range(self.joint_nums):
                    self.motorJ[i] = ModeRun(i + 1, serial=self.Serial)  # 电机ID从1开始
                    if _setzeropoint:
                        self.motorJ[i].set_zeropoint()
                self.isEnabled = _enable
            else:
                ModeRun(motor_id, serial=self.Serial)
                
    def SetDisableJoint(self, motor_id: int):
        if motor_id <= 0:
            # 所有关节使能
            for i in range(self.joint_nums):
                self.motorJ[i].disable_motor()  # 电机ID从1开始
            self.isEnabled = False
        else:
            self.motorJ[motor_id - 1].disable_motor()
            
    '''控制机械臂关节运动'''
    def SetRunJoint(self, motor_id:int, speed=1.0, pos=0):
        if self.isEnabled:
            positions = []
            velocities = []
            if motor_id <= 0:
                for i in range(self.joint_nums):
                    self.default_joint_speed[i] = speed * self.joints_run_dirction[i]
                    pos *= self.joints_run_dirction[i]
                    result, signal = self.motorJ[i].position_control_csp(
                            self.default_joint_speed[i],
                            pos)
                    joing_id = result['motor_id']
                    real_time_pos = signal['current_position']
                    real_time_speed = signal['current_velocity']
                    positions.append(real_time_pos)
                    velocities.append(real_time_speed)

                    print(f'motor_id{motor_id}: speed{speed}, pos{pos}')
                return positions, velocities
            if motor_id > 0:
                self.default_joint_speed[motor_id - 1] = speed * self.joints_run_dirction[motor_id - 1]
                pos *= self.joints_run_dirction[motor_id - 1]
                result, signal = self.motorJ[motor_id - 1].position_control_csp(
                    self.default_joint_speed[motor_id - 1],
                    pos)
                return [signal['current_position']], [signal['current_velocity']]
        else:
            print('电机未使能!')
            return None, None
        
    def SetTorque(self, motor_id: int, tq_val=7):
        if motor_id <= 0:
            for i in range(self.joint_nums):
                self.motorJ[i].SetmotorTorque(motor_id, tq_val)
                print(f'motor_id{i+1}, reset toque value: {tq_val}')
        
        else:
            self.motorJ[motor_id-1].SetmotorTorque(motor_id, tq_val)
            print(f'motor_id{motor_id}, reset toque value: {tq_val}')
                
    def SetJointSpeed(self, motor_id: int, _speed = 0):
        if motor_id <= 0:
            for i in range(self.joint_nums):
                if _speed < 0:
                    self.default_joint_speed[i] = 0
                else:
                    self.default_joint_speed[i] = _speed
        else:
            if _speed < 0:
                self.default_joint_speed[motor_id - 1] = 0;
            else:
                self.default_joint_speed[motor_id - 1] = _speed
            
    def SetJointAcceleration(self, motor_id: int, _acc=0):
        if motor_id <= 0:
            for i in range(self.joint_nums):
                if _acc < 0:
                    self.default_joint_acc[i] = 0
                else:
                    self.default_joint_acc[i] = _acc
        else:
            if _acc < 0:
                self.default_joint_acc[motor_id - 1] = 0;
            else:
                self.default_joint_acc[motor_id - 1] = _acc
            
    def SetAngle(self, motor_id: int, _angle=0):
        if motor_id <= 0:
            for i in range(self.joint_nums):
                if _angle < 0:
                    self.default_joint_angle[i] = 0
                else:
                    self.default_joint_angle[i] = _angle
        else:
            if _angle < 0:
                self.default_joint_angle[motor_id - 1] = 0;
            else:
                self.default_joint_angle[motor_id - 1] = _angle
            
    def SetMaxCurrent(self, motor_id: int, _val=0):
        if motor_id <= 0:
            for i in range(self.joint_nums):
                if _val < 0:
                    self.default_joint_current[i] = 0
                else:
                    self.default_joint_current[i] = _val
                self.motorJ[i].set_current_limit(_val)  # 设置电流限制
            time.sleep(0.5)  # 等待500ms让设置生效
        else:
            if _val < 0:
                self.default_joint_current[motor_id - 1] = 0
            else:
                self.default_joint_current[motor_id - 1] = _val
                self.motorJ[motor_id - 1].set_current_limit(_val)
            time.sleep(0.5)  # 等待500ms让设置生效
    
    '''标定关节零点位置'''        
    def CalibrateHomeOffset(self, motor_id: int, speed_val = 0.1):
        if self.find_zeropoints_flag:
            print('找零已经完成，无需重复操作')
            return
        if motor_id <= 0:
            for i in range(self.joint_nums):
                # 电机恒定速度转动寻找机械限位点
                speed_val = speed_val * self.joints_zero_dirction[i]
                self.motorJ[i].speed_control(iq_val=10, spd_ref=speed_val)
                    
                init_time = time.time() 
                while(1):
                    # 限定最大寻找时间10s
                    if (time.time() - init_time) > 6:
                        self.motorJ[i].disable_motor()
                        break
                        
                    # 根据电机速度判断是否到达限位点
                    _, result = self.motorJ[i].send_and_receive_signal(i+1)
                    if result and abs(result['current_velocity']) < 0.01:
                        self.motorJ[i].set_zeropoint()
                        self.joints_zero_point[i] = result['current_position']
                        print(f'ID: {i+1}成功设置零位，零点位置为：{result['current_position']}')
                        break
                
            self.find_zeropoints_flag = True
            print('所有关节找机械零位成功！')
                    
            return
            
    def tmp_motor_report(self, motor_id):
        if motor_id == 1:
            print("电机1自动发送数据")
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 07 e8 0c 08 64 00 11 00 00 10 0e 00 0d 0a'))
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 0f e8 0c 08 04 30 04 30 04 30 04 30 0d 0a'))
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 17 e8 0c 08 00 00 00 00 00 00 00 00 0d 0a'))
        if motor_id == 2:
            print("电机2自动发送数据")
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 07 e8 14 08 64 00 11 00 00 10 0e 00 0d 0a'))
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 0f e8 14 08 04 30 04 30 04 30 04 30 0d 0a'))
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 17 e8 14 08 00 00 00 00 00 00 00 00 0d 0a'))
        if motor_id == 3:
            print("电机3自动发送数据")
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 07 e8 1c 08 64 00 11 00 00 10 0e 00 0d 0a'))
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 0f e8 1c 08 04 30 04 30 04 30 04 30 0d 0a'))
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 17 e8 1c 08 00 00 00 00 00 00 00 00 0d 0a'))
        if motor_id == 4:
            print("电机4自动发送数据")
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 07 e8 24 08 64 00 11 00 00 10 0e 00 0d 0a'))
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 0f e8 24 08 04 30 04 30 04 30 04 30 0d 0a'))
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 17 e8 24 08 00 00 00 00 00 00 00 00 0d 0a'))
        if motor_id == 5:
            print("电机5自动发送数据")
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 07 e8 2c 08 64 00 11 00 00 10 0e 00 0d 0a'))
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 0f e8 2c 08 04 30 04 30 04 30 04 30 0d 0a'))
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 17 e8 2c 08 00 00 00 00 00 00 00 00 0d 0a'))
        if motor_id == 6:
            print("电机6自动发送数据")
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 07 e8 34 08 64 00 11 00 00 10 0e 00 0d 0a'))
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 0f e8 34 08 04 30 04 30 04 30 04 30 0d 0a'))
            self.motorJ[motor_id-1].send_data(bytes.fromhex('41 54 50 17 e8 34 08 00 00 00 00 00 00 00 00 0d 0a'))
        return
            
    def tmp_motor_stop(self):
        print('all joints stop to send message auto!')
        self.motorJ[0].send_data(bytes.fromhex('41 54 50 1f e8 0c 08 00 00 00 00 00 00 00 00 0d 0a'))
        self.motorJ[1].send_data(bytes.fromhex('41 54 50 1f e8 14 08 00 00 00 00 00 00 00 00 0d 0a'))
        self.motorJ[2].send_data(bytes.fromhex('41 54 50 1f e8 1c 08 00 00 00 00 00 00 00 00 0d 0a'))
        self.motorJ[3].send_data(bytes.fromhex('41 54 50 1f e8 24 08 00 00 00 00 00 00 00 00 0d 0a'))
        self.motorJ[4].send_data(bytes.fromhex('41 54 50 1f e8 2c 08 00 00 00 00 00 00 00 00 0d 0a'))
        self.motorJ[5].send_data(bytes.fromhex('41 54 50 1f e8 34 08 00 00 00 00 00 00 00 00 0d 0a'))

    
    def CreateRobotReport(self):
        self.motorJ[0].Serial.ClearCanData()
        for i in range(self.joint_nums):
            # reset电机
            frame = self.motorJ[i].protocol.create_motor_reset_frame(i+1)
            self.motorJ[i].send_data(frame)
            
            # frame = self.motorJ[i].protocol.create_motor_report_frame(i+1, True)
            # self.motorJ[i].send_data(frame)
            # self.tmp_motor_report(i+1)
        self.report_flag = True
        print(f'机械臂关节主动上报位置信息，可以开始接收')
    
    '''各关节回零位'''
    def Homing(self, motor_id: int, speed = 1):
        if not self.find_zeropoints_flag:
            print('error, 未设置机械零点')
            return
        if motor_id <= 0:
            for i in range(self.joint_nums):
                frame = self.motorJ[i].protocol.create_motor_write_frame( 
                    i + 1,
                    self.motorJ[i].protocol.index['LOC_REF'],
                    0.0
                )
                self.motorJ[i].send_data(frame)
                print(f'ID{i+1}回零成功')
        else:
            frame = self.motorJ[motor_id-1].protocol.create_motor_write_frame( 
                    motor_id,
                    self.motorJ[motor_id-1].protocol.index['LOC_REF'],
                    0.0
                )
            self.motorJ[motor_id-1].send_data(frame)
            print(f'ID{motor_id}回零成功')
            
    '''解析关节位置速度'''
    def ParseFeedbackFata(self, positions, velocities):
        if self.find_zeropoints_flag:
            self.rd_buffer.add_batch_data(positions, velocities)
            for i in range(self.joint_nums):
                earliest_data = self.rd_buffer.pop_earliest_data(i)
                if earliest_data is not None:
                    position, velocity = earliest_data
                    rt_pos, rt_velocitys= self.SetRunJoint(motor_id=i+1, speed=velocity, pos=position)
                    return rt_pos, rt_velocitys
                else:
                    print(f'关机{i+1}无缓存数据')
                    return None, None
        else:
            print("请先按'i'初始化机械臂位姿")
            return None, None
        
    def DeinitRobot(self, _backhome = True):
        '''结束析构'''
        self.isEnabled = False
        if _backhome:
            self.Homing(motor_id=0)
        for i in range(self.joint_nums):
            self.motorJ[i].disable_motor()
        self.Serial.ClearCanData()
        self.Serial.disconnect_serial()
        print("机械臂退出操作")
        
    def StartReportThread(self):
        """
        启动接收报文的线程
        """
        self.stop_report_thread = False
        if not self.report_flag:  # 添加条件判断，避免重复调用
            self.CreateRobotReport()
        if (self.report_flag):
            self.report_thread = threading.Thread(target=self.ReceiveReport)
            self.report_thread.start()
            print("机械臂上报线程已启动")

    def StopReportThread(self):
        """
        停止接收报文的线程
        """
        if hasattr(self, 'report_thread') and self.report_thread.is_alive():
            self.tmp_motor_stop()
            self.stop_report_thread = True
            self.report_flag = False
            self.report_thread.join()
            print("机械臂上报线程已停止")

    def ReceiveReport(self):
        """
        接收并解析报文的线程函数
        """
        while not self.stop_report_thread:
            if self.report_flag:
                try:
                    for i in range(self.joint_nums):
                        result, signal = self.motorJ[i].send_and_receive_signal(i+1)
                        if signal:
                            print(f"Joint {result['motor_id']}, mode {result['mode']}, "
                                    f"Report: Position={signal['current_position']:>7.3f}, "
                                    f"Velocity={signal['current_velocity']:>7.3f}, "
                                    f"Torque={signal['current_torque']:>7.3f}, "
                                    f"Temperature={signal['current_temperature']:>7.3f}")
                        else:
                            print("receive none")
                except Exception as e:
                    print(f"接收数据时发生错误: {str(e)}")
                    # 可以选择记录日志或采取其他恢复措施
        
        
        
        