import sys
from MainWidget import Ui_MainWidget
from PyQt5.QtWidgets import QApplication,QWidget
from PyQt5.QtGui import QIcon
from PyQt5 import QtCore
import serial.tools.list_ports
from base_protocol.motor_manager import ModeRun
from base_protocol.serial_manager import SerialConnect
from UI.robot_rander import RobotWindow,BulletWidget
from UI.PDFViewWeb import PDFViewer
import math

#获取串口列表
def get_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

print(f"串口列表：{get_serial_ports()}")


class WidgetApp(QWidget, Ui_MainWidget):
    def __init__(self):
        self.port = None
        self.serial = None
        self.motor_id = 1
        self.mode_run = None
        self.current_speed = 0
        self.speed_step = 5
        self.connect_status = False

        super(WidgetApp, self).__init__()
        self.setupUi(self)
        self.setWindowIcon(QIcon(r'./icons/DeepArm.ico'))
        self.init_com()
        self.robotwindow = RobotWindow()
        self.verticalLayout_robot.addWidget(self.robotwindow)

        self.btn_config.clicked.connect(self.on_config_clicked)
        self.btn_operate.clicked.connect(self.on_operate_clicked)
        self.btn_status.clicked.connect(self.on_status_clicked)
        self.checkBox_jog.stateChanged.connect(self.on_checkBox_jog)
        self.btn_jog_model_add.clicked.connect(self.on_jog_model_add)
        self.btn_jog_model_sub.clicked.connect(self.on_jog_model_sub)
        self.btn_com_setting_open.clicked.connect(self.on_com_setting_open)
        self.btn_init_motor.clicked.connect(self.on_init_motor)
        self.comboBox_com_setting.currentTextChanged.connect(self.on_com_setting_currentTextChanged)
        self.comboBox_com_id.currentIndexChanged.connect(self.on_com_id_currentIndexChanged)
        self.comboBox_control_model.currentTextChanged.connect(self.on_control_model_currentTextChanged)
        self.btn_stop.clicked.connect(self.on_stop_clicked)
        self.btn_instructions.clicked.connect(self.on_instructions_clicked)

        self.btn_x_pos_add.clicked.connect(self.on_x_pos_add_clicked)
        self.btn_x_pos_sub.clicked.connect(self.on_x_pos_sub_clicked)
        self.btn_y_pos_add.clicked.connect(self.on_y_pos_add_clicked)
        self.btn_y_pos_add.clicked.connect(self.on_y_pos_sub_clicked)
        self.btn_z_pos_add.clicked.connect(self.on_z_pos_add_clicked)
        self.btn_z_pos_add.clicked.connect(self.on_z_pos_sub_clicked)
        self.btn_x_angle_add.clicked.connect(self.on_x_angle_add_clicked)
        self.btn_x_angle_sub.clicked.connect(self.on_x_angle_sub_clicked)
        self.btn_y_angle_add.clicked.connect(self.on_y_angle_add_clicked)
        self.btn_y_angle_sub.clicked.connect(self.on_y_angle_sub_clicked)
        self.btn_z_angle_add.clicked.connect(self.on_z_angle_add_clicked)
        self.btn_z_angle_sub.clicked.connect(self.on_z_angle_sub_clicked)
        self.btn_send_pos_angle.clicked.connect(self.on_send_pos_angle_clicked)

        # 设置spinBox的范围，允许输入负数
        self.spinBox_x_pos.setRange(-999999, 999999)
        self.spinBox_y_pos.setRange(-999999, 999999)
        self.spinBox_z_pos.setRange(-999999, 999999)
        self.spinBox_x_angle.setRange(-999999, 999999)
        self.spinBox_y_angle.setRange(-999999, 999999)
        self.spinBox_z_angle.setRange(-999999, 999999)

        self.on_config_clicked()

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

    def update_speed(self, values=None):
        """更新速度值并返回实际速度"""
        try:
            # 如果values为None，使用当前速度
            if values is None:
                speed = self.current_speed
            else:
                # 从values字典中获取当前速度百分比
                speed = 0

            # 限制速度范围在-100到100%
            if speed < -100:
                speed = -100
            elif speed > 100:
                speed = 100

            # 更新当前速度
            self.current_speed = speed

            # 更新速度显示和滑动条
            # self.window['-SPEED-'].update(self.current_speed)
            # self.window['-SPEED_SLIDER-'].update(self.current_speed)
            self.lineEdit_current_speed.setText(str(self.current_speed)+'%')

            # 计算实际速度值
            actual_speed = int((self.current_speed / 100) * self.mode_run.V_MAX)

            # 更新电机动画
            # self.update_motor_animation(self.current_speed)

            return actual_speed

        except (ValueError, KeyError):
            self.log_message("速度值无效，请输入-100到100之间的整数")
            return 0

    def log_message(self, msg):
        self.textEdit_log.append(msg)

    def init_com(self):
        serial_ports = get_serial_ports()
        for i in serial_ports:
            self.comboBox_com_setting.addItem(i)
            self.port = serial_ports[0]

    def on_config_clicked(self):
        self.stackedWidget.setCurrentIndex(0)
        self.btn_config.setStyleSheet("#btn_config{background-color:#81b3ff; border:1px solid #dddddd; border-radius:4px; width:65; height:40px;}")
        self.btn_operate.setStyleSheet("#btn_operate{background-color:#fdfdfd; border:1px solid #dddddd; border-radius:4px; width:65; height:40px;}")
        self.btn_status.setStyleSheet("#btn_status{background-color:#fdfdfd; border:1px solid #dddddd; border-radius:4px; width:65; height:40px;}")

    def on_operate_clicked(self):
        self.stackedWidget.setCurrentIndex(1)
        self.btn_config.setStyleSheet("#btn_config{background-color:#fdfdfd; border:1px solid #dddddd; border-radius:4px; width:65; height:40px;}")
        self.btn_operate.setStyleSheet("#btn_operate{background-color:#81b3ff; border:1px solid #dddddd; border-radius:4px; width:65; height:40px;}")
        self.btn_status.setStyleSheet("#btn_status{background-color:#fdfdfd; border:1px solid #dddddd; border-radius:4px; width:65; height:40px;}")

    def on_status_clicked(self):
        self.stackedWidget.setCurrentIndex(2)
        self.btn_config.setStyleSheet("#btn_config{background-color:#fdfdfd; border:1px solid #dddddd; border-radius:4px; width:65; height:40px;}")
        self.btn_operate.setStyleSheet("#btn_operate{background-color:#fdfdfd; border:1px solid #dddddd; border-radius:4px; width:65; height:40px;}")
        self.btn_status.setStyleSheet("#btn_status{background-color:#81b3ff; border:1px solid #dddddd; border-radius:4px; width:65; height:40px;}")

    def on_com_setting_currentTextChanged(self, com):
        self.port = com

    def on_com_id_currentIndexChanged(self, motor_id):
        self.motor_id = motor_id + 1
        self.on_init_motor()
        self.log_message(f"id:{self.motor_id}")

    def on_control_model_currentTextChanged(self, model):
        self.mode = model
        if not self.connect_status:
            self.log_message("请连接串口")
            return
        try:
            if self.mode == '运控模式':
                self.log_message("运控模式暂不支持")
                return
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

    def on_com_setting_open(self):
        if self.port is None:
            self.log_message("串口未连接")
            return
        self.serial = SerialConnect(self.port)
        self.textEdit_log.append(f"串口{self.port}已经连接")

    def on_init_motor(self):
        if self.serial is None:
            self.log_message("串口未连接")
            return
        self.log_message(f"id:{self.motor_id}，com:{self.serial}！")
        self.mode_run = ModeRun(self.motor_id, self.serial)
        self.connect_status = True
        self.log_message(f"{self.port}连接成功，电机初始化成功！")

    def on_checkBox_jog(self, checked):
        if checked:
            self.log_message("打开JOG模式")
        else:
            self.log_message("关闭JOG模式")

    def on_jog_model_add(self):
        if self.checkBox_jog.isChecked():
            direction = "CW"
            self.jog_run(direction)
        else:
            self.log_message("请先选择JOG模式")

    def on_jog_model_sub(self):
        if self.checkBox_jog.isChecked():
            direction = "CCW"
            self.jog_run(direction)
        else:
            self.log_message("请先选择JOG模式")

    def on_stop_clicked(self):
        if self.mode_run:
            self.mode_run.jog_stop()
            self.mode_run.disable_motor()
            self.log_message("电机已停止")

    def on_instructions_clicked(self):
        self.pdfView = PDFViewer('./docs/产品资料/RS00/RS00使用说明书250227.pdf')
        self.pdfView.show()

    def on_x_pos_add_clicked(self):
        pos1 = float(self.lineEdit_x_pos.text())
        pos2 = float(self.spinBox_x_pos.value())
        self.lineEdit_x_pos.setText(str(round(pos1 + pos2, 3)))
        self.send_pos_or_angle(1)

    def on_x_pos_sub_clicked(self):
        pos1 = float(self.lineEdit_x_pos.text())
        pos2 = float(self.spinBox_x_pos.value())
        self.lineEdit_x_pos.setText(str(round(pos1 - pos2, 3)))
        self.send_pos_or_angle(2)

    def on_y_pos_add_clicked(self):
        pos1 = float(self.lineEdit_y_pos.text())
        pos2 = float(self.spinBox_y_pos.value())
        self.lineEdit_y_pos.setText(str(round(pos1 + pos2, 3)))
        self.send_pos_or_angle(3)

    def on_y_pos_sub_clicked(self):
        pos1 = float(self.lineEdit_y_pos.text())
        pos2 = float(self.spinBox_y_pos.value())
        self.lineEdit_y_pos.setText(str(round(pos1 - pos2, 3)))
        self.send_pos_or_angle(4)

    def on_z_pos_add_clicked(self):
        pos1 = float(self.lineEdit_z_pos.text())
        pos2 = float(self.spinBox_z_pos.value())
        self.lineEdit_z_pos.setText(str(round(pos1 + pos2, 3)))
        self.send_pos_or_angle(5)

    def on_z_pos_sub_clicked(self):
        pos1 = float(self.lineEdit_z_pos.text())
        pos2 = float(self.spinBox_z_pos.value())
        self.lineEdit_z_pos.setText(str(round(pos1 - pos2, 3)))
        self.send_pos_or_angle(6)

    def on_x_angle_add_clicked(self):
        pos1 = float(self.lineEdit_x_angle.text())
        pos2 = float(self.spinBox_x_angle.value())
        self.lineEdit_x_angle.setText(str(round(pos1 + pos2, 3)))
        self.send_pos_or_angle(7)

    def on_x_angle_sub_clicked(self):
        pos1 = float(self.lineEdit_y_angle.text())
        pos2 = float(self.spinBox_y_angle.value())
        self.lineEdit_y_angle.setText(str(round(pos1 - pos2, 3)))
        self.send_pos_or_angle(8)

    def on_y_angle_add_clicked(self):
        pos1 = float(self.lineEdit_y_angle.text())
        pos2 = float(self.spinBox_y_angle.value())
        self.lineEdit_y_angle.setText(str(round(pos1 + pos2, 3)))
        self.send_pos_or_angle(9)

    def on_y_angle_sub_clicked(self):
        pos1 = float(self.lineEdit_y_angle.text())
        pos2 = float(self.spinBox_y_angle.value())
        self.lineEdit_y_angle.setText(str(round(pos1 - pos2, 3)))
        self.send_pos_or_angle(10)

    def on_z_angle_add_clicked(self):
        pos1 = float(self.lineEdit_z_angle.text())
        pos2 = float(self.spinBox_z_angle.value())
        self.lineEdit_z_angle.setText(str(round(pos1 + pos2, 3)))
        self.send_pos_or_angle(11)

    def on_z_angle_sub_clicked(self):
        pos1 = float(self.lineEdit_z_angle.text())
        pos2 = float(self.spinBox_z_angle.value())
        self.lineEdit_z_angle.setText(str(round(pos1 - pos2, 3)))
        self.send_pos_or_angle(12)

    def on_send_pos_angle_clicked(self):
        posX = round(float(self.spinBox_x_pos.text()), 3)
        posY = round(float(self.spinBox_y_pos.text()), 3)
        posZ = round(float(self.spinBox_z_pos.text()), 3)
        angleR = round(float(self.spinBox_x_angle.text()), 3)
        angleP = round(float(self.spinBox_y_angle.text()), 3)
        angleY = round(float(self.spinBox_z_angle.text()), 3)
        self.label_tips.setText(f"发送的数据：\nX:{posX}, Y:{posY}, Z:{posZ}, \nRx:{angleR}, Ry:{angleP}, Rz:{angleY}")
        self.robotwindow.setRobotPosAngle(posX, posY, posZ, angleR, angleP, angleY)

    def send_pos_or_angle(self, type):
        target_pos , target_rpy = self.robotwindow.getRobotPosAngle()
        if type == 1: #X+
            target_pos[0] += (1 / 100)
        elif type == 2: #X-
            target_pos[0] += (-1 / 100)
        elif type == 3: #Y+
            target_pos[1] += (1 / 100)
        elif type == 4: #Y-
            target_pos[1] += (-1 / 100)
        elif type == 5: #Z+
            target_pos[2] += (1 / 100)
        elif type == 6: #Z-
            target_pos[2] += (-1 / 100)
        elif type == 7: #Rx+
            target_rpy[0] += math.radians(1)
        elif type == 8: #Rx-
            target_rpy[0] += math.radians(-1)
        elif type == 9: #Ry+
            target_rpy[1] += math.radians(1)
        elif type == 10: #Ry-
            target_rpy[1] += math.radians(-1)
        elif type == 11: #Rz+
            target_rpy[2] += math.radians(1)
        elif type == 12: #Rz-
            target_rpy[2] += math.radians(-1)
        self.label_tips.setText(f"发送的数据：\nX:{target_pos[0]}, Y:{target_pos[1]}, Z:{target_pos[2]}, \nRx:{target_rpy[0]}, Ry:{target_rpy[1]}, Rz:{target_rpy[2]}")
        self.robotwindow.setRobotPosAngle(target_pos[0], target_pos[1], target_pos[2], target_rpy[0], target_rpy[1], target_rpy[2])


if __name__ == "__main__":
    QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    mw = WidgetApp()
    mw.showMaximized()
    sys.exit(app.exec_())