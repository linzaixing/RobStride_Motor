import PySimpleGUI as sg
from base_protocol.motor_manager import ModeRun
import math
from base_protocol.serial_manager import SerialConnect

class MotorControlApp:
    def __init__(self, mode_id=1):
        self.motor_id = mode_id
        self.mode_run = None
        self.current_speed = 0
        self.speed_step = 5
        self.serial = None
        
        self.connect_status = False
        self.JOG_status = False
        self.mode = None
        
        # 修改电机动画元素
        self.motor_canvas = sg.Graph(
            canvas_size=(200, 200),
            graph_bottom_left=(0, 0),
            graph_top_right=(200, 200),
            key='-MOTOR-',
            background_color='white'
        )
        
        # 先定义布局
        self.layout = [
            [sg.Text('端口号:'), sg.Combo(['COM1', 'COM2', 'COM3', 'COM4', 'COM5'], 
                                     default_value='COM5',
                                     key='-PORT-', size=(10, 1))],
            [sg.Text('电机ID:'), sg.Combo(['1', '2', '3', '4', '5', '6'], 
                                     default_value='1',
                                     key='-ID-', size=(10, 1))],
            [sg.Button('连接串口', size=(8, 1)), sg.Button('初始化电机', size=(8, 1))],
            [sg.Button('JOG模式', size=(8, 1)), sg.Button('+', size=(3, 1)), sg.Button('-', size=(3, 1))],
            [sg.Text('控制模式:'), sg.Combo(['运控模式', '位置模式(PP)', '速度模式', '电流模式', '位置模式(CSP)'], 
                                      default_value='速度模式',
                                      key='-MODE-', size=(15, 1))],
            [sg.Text('当前速度:'), sg.Text('0', key='-SPEED-', size=(5, 1)), sg.Text('%')],
            [sg.Slider(range=(-100, 100), default_value=0, orientation='h', 
                       size=(30, 15), key='-SPEED_SLIDER-', enable_events=True)],
            [self.motor_canvas],  # 添加电机动画
            [sg.Button('启动当前模式', size=(8, 1)), sg.Button('停止当前模式', size=(8, 1))],
            [sg.Multiline('', size=(40, 10), key='-LOG-', autoscroll=True, disabled=True)],
            [sg.Button('退出')]
        ]
        
        # 然后创建窗口
        self.window = sg.Window('电机控制', self.layout, finalize=True, icon=r'RobStride_Motor\icons\steppermotor_5459.ico')
        
        # 现在可以安全地进行图形操作
        self.motor_circle = self.motor_canvas.draw_circle(
            (100, 100), 80,
            fill_color='none',
            line_color='black',
            line_width=3
        )
        self.motor_center = self.motor_canvas.draw_circle(
            (100, 100), 50,
            fill_color='black',
            line_color='black',
            line_width=1
        )
        # self.motor_arrow = self.motor_canvas.draw_line(
        #     (100, 100), 
        #     (100, 150),
        #     width=3, 
        #     color='red',
        #     arrow='last'
        # )
        # 添加角度指示
        for angle in range(0, 360, 30):
            x1 = 100 + 70 * math.cos(math.radians(angle))
            y1 = 100 + 70 * math.sin(math.radians(angle))
            x2 = 100 + 80 * math.cos(math.radians(angle))
            y2 = 100 + 80 * math.sin(math.radians(angle))
            self.motor_canvas.draw_line((x1, y1), (x2, y2), width=1, color='black')
        self.rotation_angle = 0

    def jog_run(self, direction):
        """更新速度值"""
        if not self.mode_run:
            self.log_message("请先初始化电机")
            return
            
        self.mode_run.jog_mode_active = not self.mode_run.jog_mode_active
        self.mode_run.speed_mode_active = False
        self.mode_run.iq_mode_active = False
        if self.mode_run.jog_mode_active:
            if direction == "CW":
                self.log_message("进入JOG模式: CW")
                self.mode_run.jog_control("CW")
            else:
                self.log_message("进入JOG模式: CCW")
                self.mode_run.jog_control("CCW")
        else:
            self.mode_run.jog_stop()
            self.mode_run.disable_motor()
            # self.mode_run.enable_motor()
            self.log_message("退出JOG模式")

    def log_message(self, message):
        """记录日志信息"""
        self.window['-LOG-'].update(message + '\n', append=True)
    
    def update_motor_animation(self, speed):
        """更新电机动画"""
        # 计算旋转角度
        rotation_speed = abs(speed) * 0.36  # 每1%速度对应0.36度
        if speed < 0:
            rotation_speed *= -1  # 反转方向
            
        # 更新箭头位置
        self.rotation_angle = (self.rotation_angle + rotation_speed) % 360
        self.motor_canvas.delete_figure(self.motor_arrow)
        self.motor_arrow = self.motor_canvas.draw_line(
            (100, 100),
            (100 + 50 * math.cos(math.radians(self.rotation_angle)),
             100 + 50 * math.sin(math.radians(self.rotation_angle))),
            width=3,
            color='red',
            arrow='last'
        )

    def update_speed(self, values=None):
        """更新速度值并返回实际速度"""
        try:
            # 如果values为None，使用当前速度
            if values is None:
                speed = self.current_speed
            else:
                # 从values字典中获取当前速度百分比
                speed = values['-SPEED_SLIDER-']
            
            # 限制速度范围在-100到100%
            if speed < -100:
                speed = -100
            elif speed > 100:
                speed = 100
                
            # 更新当前速度
            self.current_speed = speed
                
            # 更新速度显示和滑动条
            self.window['-SPEED-'].update(self.current_speed)
            self.window['-SPEED_SLIDER-'].update(self.current_speed)
            
            # 计算实际速度值
            actual_speed = int((self.current_speed / 100) * self.mode_run.V_MAX)
            
            # 更新电机动画
            # self.update_motor_animation(self.current_speed)
            
            return actual_speed
            
        except (ValueError, KeyError):
            self.log_message("速度值无效，请输入-100到100之间的整数")
            return 0

    def run(self):
        while True:
            event, values = self.window.read(timeout=100)  # 添加timeout参数以支持动画更新

            if event == sg.WIN_CLOSED or event == '退出':
                if self.mode_run:
                    self.mode_run.jog_stop()
                    self.mode_run.disable_motor()
                    self.log_message("电机已停止")
                break
            
            elif event == '-SPEED_SLIDER-':
                if self.mode_run:
                    self.update_speed(values)
                    
            elif event == '连接串口':
                port = values['-PORT-']
                if not port:
                    self.log_message("串口未连接")
                    continue
                self.serial = SerialConnect(port)
                self.log_message(f"串口{port}已经连接")
            
            elif event == '初始化电机':
                motor_id = int(values['-ID-'])
                if not self.serial:
                    self.log_message("串口未连接")
                    continue
                self.mode_run = ModeRun(motor_id, self.serial)
                self.connect_status = True
                self.log_message(f"port:{port}连接成功，电机初始化成功！")
            
            elif event == 'JOG模式':
                if not self.connect_status:
                    self.log_message("端口连接失败！")
                    continue
                self.JOG_status = not self.JOG_status
                self.log_message(f"JOG模式 {'启动' if self.JOG_status else '停止'}")
                    
            elif event == '+':
                if self.JOG_status:
                    direction = "CW"
                    self.jog_run(direction)
                else:
                    self.log_message("请先选择JOG模式")

            elif event == '-':
                if self.JOG_status:
                    direction = "CCW"
                    self.jog_run(direction)
                else:
                    self.log_message("请先选择JOG模式")

            elif event == '启动当前模式':
                self.mode = values['-MODE-']
                if not self.connect_status:
                    self.log_message("请连接串口")
                    continue
                try:
                    if self.mode == '运控模式':
                        self.log_message("运控模式暂不支持")
                        continue
                    elif self.mode == '位置模式(PP)':
                        self.log_message("位置模式(PP)")
                        speed_val = self.update_speed()
                        self.mode_run.position_control(speed_val=speed_val)
                    elif self.mode == '位置模式(CSP)':
                        self.log_message("位置模式(CSP)")
                        speed_val = self.update_speed()
                        self.mode_run.position_control_csp(speed_val=speed_val)
                    elif self.mode == '速度模式':
                        self.mode_run.speed_mode_active = True
                        self.mode_run.jog_mode_active = False
                        self.mode_run.iq_mode_active = False
                        speed_val = self.update_speed()
                        self.mode_run.speed_control(spd_ref=speed_val)
                        self.log_message(f"速度模式已启动，速度: {self.current_speed}%")
                    elif self.mode == '电流模式':
                        iq_val = self.update_speed()
                        self.mode_run.iq_control(iq_val=iq_val)
                        self.log_message(f"电流模式已启动，速度: {self.current_speed}%")
                
                except Exception as e:
                    self.log_message(f"启动失败: {str(e)}")

            elif event == '停止当前模式':
                if self.mode_run:
                    self.mode_run.jog_stop()
                    self.mode_run.disable_motor()
                    self.log_message("电机已停止")

            # # 持续更新电机动画
            # if self.mode_run and (self.mode_run.speed_mode_active or self.mode_run.jog_mode_active):
            #     self.update_motor_animation(self.current_speed)

        self.window.close()

if __name__ == "__main__":
    app = MotorControlApp()
    app.run()