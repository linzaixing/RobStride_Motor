import sys
import numpy as np
import pybullet as p
import pybullet_data
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QSlider,
    QLabel, QPushButton  # 确保包含QPushButton
)

class BulletWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(1280, 960)

        # 初始化UI布局
        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setMouseTracking(True)  # 启用鼠标跟踪
        self.label.setAttribute(Qt.WA_TransparentForMouseEvents)  # 关键修复：鼠标事件穿透
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.label)
        self.setLayout(layout)

        # PyBullet初始化
        self.physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        print(pybullet_data.getDataPath())
        self.planeId = p.loadURDF(r".\docs\arm_07_urdf\urdf\plane.urdf")
        p.setGravity(0, 0, -9.81)  # 添加重力设置

        # 加载机械臂模型（添加错误检查）
        try:
            self.robotId = p.loadURDF(r".\docs\arm_07_urdf\urdf\arm_07_urdf.urdf", [0, 0, 0], useFixedBase=1)
            self.num_joints = p.getNumJoints(self.robotId)
            # 手动设置颜色
            for i in range(self.num_joints):
                # 获取链接信息
                link_info = p.getVisualShapeData(self.robotId, i)
                
                # 设置颜色 (RGBA格式)
                p.changeVisualShape(self.robotId, i, rgbaColor=[0.792, 0.820, 0.933, 1])  # 示例颜色

        except:
            raise ValueError("无法加载URDF文件，请检查路径是否正确")

        # 相机参数初始化
        self.camera_distance = 2.0
        self.camera_yaw = 15
        self.camera_pitch = -30
        self.camera_target = np.array([-0.1, 0.2, 0.0])

        # 定时器设置
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 30 FPS

        # 鼠标参数
        self.last_mouse_pos = None
        self.mouse_sensitivity = 0.2

    def update_frame(self):
        # 物理仿真步进
        p.stepSimulation()  # 关键修复：添加物理更新

        # 获取渲染尺寸
        width = self.width()
        height = self.height()

        # 计算视图矩阵
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            self.camera_target,
            self.camera_distance,
            self.camera_yaw,
            self.camera_pitch,
            0,
            2
        )

        # 计算投影矩阵
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=30,
            aspect=width/height,
            nearVal=0.1,
            farVal=100
        )

        # 获取相机图像（修复图像方向）
        _, _, rgb, depth, seg = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        # 转换图像格式（关键修复：处理BGR到RGB的转换）
        rgb = np.reshape(rgb, (height, width, 4))[:, :, :3]
        rgb = np.ascontiguousarray(rgb[..., ::-1])  # BGR -> RGB

        # 创建QImage并显示
        q_img = QImage(
            rgb.data,
            width,
            height,
            3 * width,  # bytesPerLine
            QImage.Format_RGB888
        )
        self.label.setPixmap(QPixmap.fromImage(q_img))

    def mousePressEvent(self, event):
        self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self.last_mouse_pos:
            dx = event.x() - self.last_mouse_pos.x()
            dy = event.y() - self.last_mouse_pos.y()

            # 左键旋转视角
            if event.buttons() == Qt.LeftButton:
                self.camera_yaw -= dx * self.mouse_sensitivity
                self.camera_pitch = np.clip(
                    self.camera_pitch + dy * self.mouse_sensitivity,
                    -89, 89
                )

            # 右键平移视角
            elif event.buttons() == Qt.RightButton:
                right = np.array([np.cos(np.deg2rad(self.camera_yaw)), 0, np.sin(np.deg2rad(self.camera_yaw))])
                up = np.array([0, 0, 1])
                forward = np.cross(right, up)

                self.camera_target -= right * dx * 0.01
                self.camera_target -= forward * dy * 0.01

            self.last_mouse_pos = event.pos()

    def wheelEvent(self, event):
        self.camera_distance = np.clip(
            self.camera_distance - event.angleDelta().y() * 0.001,
            0.5, 5
        )

class RobotWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("机械臂控制平台")
        self.setGeometry(100, 100, 1920, 1080)

        # 主布局
        central_widget = QWidget()
        # self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        self.setLayout(layout)

        # 3D显示区域
        self.bullet_widget = BulletWidget()
        layout.addWidget(self.bullet_widget)
        layout.addSpacing(10)

        # 控制面板
        control_panel = QWidget()
        control_layout = QHBoxLayout(control_panel)
        layout.addWidget(control_panel)

        # 关节控制滑块
        self.joint_sliders = []
        for i in range(self.bullet_widget.num_joints):
            slider = QSlider(Qt.Vertical)
            slider.setRange(-180, 180)
            slider.setValue(0)
            slider.setPageStep(5)
            slider.setFixedHeight(100)  # 设置滑动条高度
            slider.valueChanged.connect(
                lambda value, idx=i: self.set_joint_angle(idx, value)
            )

            control_layout.addWidget(QLabel(f"关\n节\n{i+1}"))
            control_layout.addWidget(slider)
            control_layout.addSpacing(30)
            self.joint_sliders.append(slider)

        # 添加复位按钮
        reset_btn = QPushButton("复位")
        reset_btn.setFixedWidth(35)
        reset_btn.clicked.connect(self.reset_joints)
        control_layout.addWidget(reset_btn)
        control_layout.addSpacing(20)

    def set_joint_angle(self, joint_idx, angle):
        p.setJointMotorControl2(
            self.bullet_widget.robotId,
            joint_idx,
            p.POSITION_CONTROL,
            targetPosition=np.deg2rad(angle),
            targetVelocity=0,  # 目标速度
            positionGain=0.5,  # 位置增益
            velocityGain=1.0,  # 速度增益
            force=500
        )

    def reset_joints(self):
        for slider in self.joint_sliders:
            slider.setValue(0)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotWindow()
    window.show()
    sys.exit(app.exec_())