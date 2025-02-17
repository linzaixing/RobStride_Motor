
import pygame
import time
import motor_manager

class ps5_controller:
    """PS5遥控器模式控制类"""
    
    def __init__(self, motor_id: int):
        """初始化PS5手柄控制相关变量"""
        self.motor_id = motor_id
        self.motor_ctr = motor_manager.ModeRun(self.motor_id)
        self.jog_mode_active = self.motor_ctr.jog_mode_active
        self.speed_mode_active = self.motor_ctr.speed_mode_active
        self.iq_mode_active = self.motor_ctr.iq_mode_active
        self.iq_speed_activate = self.motor_ctr.iq_speed_activate
        self.jog_speed_cw = self.motor_ctr.jog_speed_cw
        self.jog_speed_ccw = self.motor_ctr.jog_speed_ccw
        
    def handle_ps5_input(self) -> None:
        """处理PS5手柄输入"""
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            print("未检测到PS5手柄")
            return

        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        print("PS5手柄已连接")

        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0:
                        print('按钮0: 叉按钮-->停止所有运动')
                        self.motor_ctr.stop_motion()
                        self.motor_ctr.jog_stop()
                        self.jog_mode_active = False  # 标记JOG模式是否激活
                        self.speed_mode_active = False  # 标记速度模式是否激活
                        self.iq_mode_active = False  # 标记电流模式是否激活
                    elif event.button == 1:
                        print('按钮1: 圆形按钮-->电流模式')
                        self.iq_mode_active = not self.iq_mode_active
                        self.jog_mode_active = False
                        self.speed_mode_active = False
                        if self.iq_mode_active:
                            print("进入电流模式")
                        else:
                            self.motor_ctr.stop_motion()
                            print("退出电流模式")
                    elif event.button == 2:
                        print('按钮2: 方形按钮-->位置模式 PP')
                        self.motor_ctr.position_control()
                    elif event.button == 3:
                        print('按钮3: 三角按钮-->速度模式')
                        self.speed_mode_active = not self.speed_mode_active
                        self.jog_mode_active = False
                        self.iq_mode_active = False
                        if self.speed_mode_active:
                            print("进入速度模式")
                        else:
                            self.motor_ctr.stop_motion()
                            print("退出速度模式")
                    elif event.button == 4:
                        print('按钮4: SHARE按钮')
                    elif event.button == 5:
                        print('按钮5: PS按钮-->退出程序')
                        self.motor_ctr.stop_motion()
                        self.motor_ctr.jog_stop()
                        pygame.quit()
                        return
                    elif event.button == 6:
                        print('按钮6: OPTION按钮-->CSP位置模式')
                        self.motor_ctr.position_control_csp()
                    elif event.button == 7:
                        print('按钮7: 左摇杆中键-->JOG模式')
                        self.jog_mode_active = not self.jog_mode_active
                        self.speed_mode_active = False
                        self.iq_mode_active = False
                        if self.jog_mode_active:
                            print("进入JOG模式")
                        else:
                            self.motor_ctr.jog_stop()
                            print("退出JOG模式")
                    elif event.button == 8:
                        print('按钮8: 右摇杆中键')
                        if self.speed_mode_active or self.iq_mode_active:
                            self.iq_speed_activate = not self.iq_speed_activate
                            if self.iq_speed_activate and self.speed_mode_active:
                                self.motor_ctr.speed_control()
                            if self.iq_speed_activate and self.iq_mode_active:
                                self.motor_ctr.iq_control()
                    elif event.button == 9:
                        print('按钮9: L1按钮')
                        if self.jog_mode_active:
                            self.jog_speed_cw = min(self.jog_speed_cw + 0x100, 0xF000)  # 限制最大速度
                            self.jog_speed_ccw = max(self.jog_speed_ccw - 0x100, 0x8000)  # 限制最小速度
                            print(f"JOG速度增加至: 正转 {self.jog_speed_cw}, 反转 {self.jog_speed_ccw}")
                
                    elif event.button == 10:
                        print('按钮10: R1按钮')
                        if self.jog_mode_active:
                            self.jog_speed_cw = max(self.jog_speed_cw - 0x100, 0x111)  # 限制最小速度
                            self.jog_speed_ccw = min(self.jog_speed_ccw + 0x100, 0xFFFF)  # 限制最大速度
                            print(f"JOG速度减少至: 正转 {self.jog_speed_cw}, 反转 {self.jog_speed_ccw}")
     
            # 如果JOG模式激活，读取摇杆值
            if self.jog_mode_active:
                # 获取左摇杆的X轴值（通常为axis 0）
                axis_value = joystick.get_axis(0)
                if axis_value > 0.5:  # 右摇
                    self.motor_ctr.jog_control("CW", speed_value=self.jog_speed_cw)
                elif axis_value < -0.5:  # 左摇
                    self.motor_ctr.jog_control("CCW", speed_value=self.jog_speed_ccw)
                else:  # 摇杆回中
                    self.motor_ctr.jog_stop()
                    
            # 如果速度模式激活，读取右摇杆值
            if self.speed_mode_active:
                if self.iq_speed_activate:
                    continue
                axis_value = joystick.get_axis(2)  # 右摇杆X轴
                speed_val = int(axis_value * self.motor_ctr.V_MAX)
                if abs(axis_value) > 0.1:
                    self.motor_ctr.speed_control(spd_ref=speed_val)
                else:
                    self.motor_ctr.stop_motion()
                    
            # 如果电流模式激活，读取右摇杆值
            if self.iq_mode_active:
                if self.iq_speed_activate:
                    continue
                axis_value = joystick.get_axis(2)  # 右摇杆X轴
                iq_val = int(axis_value * 16)  # 假设最大电流为10A
                if abs(axis_value) > 0.1:
                    self.motor_ctr.iq_control(iq_val=iq_val)
                else:
                    self.motor_ctr.stop_motion()
                    
            time.sleep(0.1)  # 防止CPU占用过高

    