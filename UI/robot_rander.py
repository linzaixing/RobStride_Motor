import sys
import numpy as np
import pybullet as p
import pybullet_data
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QFont

# from robot_module.robot_kinamatic import PybulletRobot

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QSlider,
    QLabel, QPushButton  # 确保包含QPushButton
)
import math
import time

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
        
        # 新增文字标签缓存
        self.axis_labels = {
            'X': {'pos3d': None, 'color': (0, 0, 255)},
            'Y': {'pos3d': None, 'color': (0, 255, 0)},
            'Z': {'pos3d': None, 'color': (255, 0, 0)}
        }

        # PyBullet初始化
        self.physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        print(pybullet_data.getDataPath())
        self.planeId = p.loadURDF(r"..\docs\arm_07_urdf\urdf\plane.urdf")
        p.setGravity(0, 0, -9.81)  # 添加重力设置

        # 加载机械臂模型（添加错误检查）
        try:
            self.robotId = p.loadURDF(r"..\docs\arm_07_urdf\urdf\arm_07_urdf.urdf", [0, 0, 0], useFixedBase=1)
            self.num_joints = p.getNumJoints(self.robotId)
            # 获取基座位置和方向
            self.base_pos, self.base_ori = p.getBasePositionAndOrientation(self.robotId)
            self.create_coordinate_axes()

            # 定义不同关节的颜色
            colors = [
                [0.5, 0.5, 0.5, 1.0],   # 灰色
                [1.0, 1.0, 1.0, 1.0],  # 白色
                [0.5, 0.5, 0.5, 1.0],   # 灰色
                [1.0, 1.0, 1.0, 1.0],  # 白色
                [0.5, 0.5, 0.5, 1.0],   # 灰色
                [1.0, 1.0, 1.0, 1.0],  # 白色
                [0.5, 0.5, 0.5, 1.0]   # 灰色
            ]
            
            # 为每个关节设置不同颜色
            for i in range(-1, self.num_joints):  # 从-1开始，包括基座
                color_index = (i + 1) % len(colors)  # 循环使用颜色列表
                p.changeVisualShape(self.robotId, i, 
                                    rgbaColor=colors[color_index])

        except:
            raise ValueError("无法加载URDF文件，请检查路径是否正确")

        # 相机参数初始化
        self.camera_distance = 2.0
        self.camera_yaw = 35
        self.camera_pitch = -30
        self.camera_target = np.array([-0.1, 0.2, 0.0])

        # 定时器设置
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 30 FPS

        # 鼠标参数
        self.last_mouse_pos = None
        self.mouse_sensitivity = 0.2
        
        self.getRobotPosAngle()

        # 计算逆运动学
        target_quat = p.getQuaternionFromEuler(self.target_rpy)
        joint_angles = p.calculateInverseKinematics(
            self.robotId,
            self.last_joint_index,
            self.target_pos,
            target_quat,
            maxNumIterations=100,
            residualThreshold=1e-5,
            jointDamping=[0.1] * 6
            # jointIndices=controlled_joints
        )
        print('\n move target_pos: ', self.target_pos)
        print('\n move target_rpy: ', self.target_rpy)
        print('joint_angles:', joint_angles)

        # 应用关节控制
        for i, joint_index in enumerate(self.controlled_joints):
            p.setJointMotorControl2(
                self.robotId,
                joint_index,
                p.POSITION_CONTROL,
                targetPosition=joint_angles[i],
                force=500
            )
    
    def create_coordinate_axes(self):
        """创建固定在地面的坐标轴系统"""
        axis_length = 0.5
        axis_radius = 0.01
        arrow_length = 0.001
        arrow_radius = 0.02
        
        # X轴（红色）
        x_axis = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=axis_radius,
            length=axis_length,
            rgbaColor=[0, 0, 1, 1]
        )
        x_arrow = p.createVisualShape(
            p.GEOM_CAPSULE,
            radius=arrow_radius,
            length=arrow_length,
            rgbaColor=[0, 0, 1, 1]
        )
        p.createMultiBody(
            baseVisualShapeIndex=x_axis,
            basePosition=[self.base_pos[0] + axis_length/2, self.base_pos[1], self.base_pos[2]],
            baseOrientation=p.getQuaternionFromEuler([0, np.pi/2, 0])
        )
        p.createMultiBody(
            baseVisualShapeIndex=x_arrow,
            basePosition=[self.base_pos[0] + axis_length + arrow_length/2, self.base_pos[1], self.base_pos[2]],
            baseOrientation=p.getQuaternionFromEuler([0, np.pi/2, 0])
        )
        
        # Y轴（绿色）
        y_axis = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=axis_radius,
            length=axis_length,
            rgbaColor=[0, 1, 0, 1]
        )
        y_arrow = p.createVisualShape(
            p.GEOM_CAPSULE,
            radius=arrow_radius,
            length=arrow_length,
            rgbaColor=[0, 1, 0, 1]
        )
        p.createMultiBody(
            baseVisualShapeIndex=y_axis,
            basePosition=[self.base_pos[0], self.base_pos[1] + axis_length/2, self.base_pos[2]],
            baseOrientation=p.getQuaternionFromEuler([np.pi/2, 0, 0])
        )
        p.createMultiBody(
            baseVisualShapeIndex=y_arrow,
            basePosition=[self.base_pos[0], self.base_pos[1] + axis_length + arrow_length/2, self.base_pos[2]],
            baseOrientation=p.getQuaternionFromEuler([np.pi/2, 0, 0])
        )
        
        # Z轴（蓝色）
        z_axis = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=axis_radius,
            length=axis_length,
            rgbaColor=[1, 0, 0, 1]
        )
        z_arrow = p.createVisualShape(
            p.GEOM_CAPSULE,
            radius=arrow_radius,
            length=arrow_length,
            rgbaColor=[1, 0, 0, 1]
        )
        p.createMultiBody(
            baseVisualShapeIndex=z_axis,
            basePosition=[self.base_pos[0], self.base_pos[1], self.base_pos[2] + axis_length/2]
        )
        p.createMultiBody(
            baseVisualShapeIndex=z_arrow,
            basePosition=[self.base_pos[0], self.base_pos[1], self.base_pos[2] + axis_length + arrow_length/2],
            baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi/2])
        )
        
        # 记录标签的3D位置（去掉箭头长度部分）
        self.axis_labels['X']['pos3d'] = [self.base_pos[0] + axis_length, self.base_pos[1], self.base_pos[2]]
        self.axis_labels['Y']['pos3d'] = [self.base_pos[0], self.base_pos[1] + axis_length, self.base_pos[2]]
        self.axis_labels['Z']['pos3d'] = [self.base_pos[0], self.base_pos[1], self.base_pos[2] + axis_length]

    def update_frame(self):
        # 物理仿真步进
        p.stepSimulation()

        width = self.width()
        height = self.height()

        # 获取并重塑视图矩阵（列主序）
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            self.camera_target,
            self.camera_distance,
            self.camera_yaw,
            self.camera_pitch,
            0, 2
        )
        view_matrix = np.array(view_matrix).reshape(4, 4).T

        # 获取并重塑投影矩阵（列主序）
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=30,
            aspect=width/height,
            nearVal=0.1,
            farVal=100
        )
        proj_matrix = np.array(proj_matrix).reshape(4, 4).T

        # 获取相机图像
        _, _, rgb, _, _ = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_matrix.flatten('F').tolist(),  # 保持列主序
            projectionMatrix=proj_matrix.flatten('F').tolist(),
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        # 转换图像格式
        rgb = np.reshape(rgb, (height, width, 4))[:, :, :3]
        rgb = np.ascontiguousarray(rgb[..., ::-1])  # BGR -> RGB
        
        # 创建QImage
        q_img = QImage(
            rgb.data,
            1280,
            960,
            QImage.Format_BGR888
        )

        self.addText(q_img, view_matrix, proj_matrix, width, height)
        self.label.setPixmap(QPixmap.fromImage(q_img))
        
    def addText(self, q_img, view_matrix, proj_matrix, width, height):
        # 创建QPainter绘制文字
        painter = QPainter(q_img)
        painter.setRenderHint(QPainter.Antialiasing)
        font = QFont("Arial", 28, QFont.Bold)
        painter.setFont(font)

        for label, data in self.axis_labels.items():
            pos3d = data['pos3d']
            # 转换为齐次坐标（列向量）
            pos3d_homogeneous = np.array([pos3d[0], pos3d[1], pos3d[2], 1.0]).reshape(4, 1)

            # 视图变换
            view_space = np.dot(view_matrix, pos3d_homogeneous)
            
            # 投影变换
            clip_space = np.dot(proj_matrix, view_space)
            
            # 透视除法
            if clip_space[3] == 0:
                continue
            ndc = clip_space[:3] / clip_space[3]

            # 转换为屏幕坐标
            x = int((ndc[0] + 1) * width / 2)
            y = int((1 - ndc[1]) * height / 2)

            # 安全限制
            x = np.clip(x, 0, width-1)
            y = np.clip(y, 0, height-1)

            # 绘制文字
            painter.setPen(QColor(0, 0, 0, 128))
            painter.drawText(x + 2, y + 2, label)
            painter.setPen(QColor(*data['color']))
            painter.drawText(x, y, label)

        painter.end()

    def getRobotPosAngle(self):
        # 获取可动关节索引（过滤固定关节）
        joint_indices = [i for i in range(p.getNumJoints(self.robotId)) if p.getJointInfo(self.robotId, i)[2] != p.JOINT_FIXED]
        print('joint_indices: ', len(joint_indices))
        # assert len(joint_indices) >= 6, "机械臂需要至少6个可动关节"
        if len(joint_indices) < 6:
            return None
        self.controlled_joints = joint_indices[:6]  # 取前6个可动关节

        # 获取末端执行器链接索引（最后一个控制关节的子链接）
        self.last_joint_index = self.controlled_joints[-1]
        joint_info = p.getJointInfo(self.robotId, self.last_joint_index)
        self.end_effector_link_index = joint_info[16]  # 使用jointInfo的childLinkIndex字段

        # 保存初始关节位置
        self.initial_joint_positions = [p.getJointState(self.robotId, i)[0] for i in self.controlled_joints]

        # 初始化目标位置和姿态（基于当前末端状态）
        link_state = p.getLinkState(self.robotId, self.end_effector_link_index)
        # assert link_state is not None, "无法获取末端执行器状态，请检查链接索引"
        if link_state is None:
            return None
        current_end_pos, current_end_orn = link_state[:2]
        self.target_pos = list(current_end_pos)
        self.target_rpy = list(p.getEulerFromQuaternion(current_end_orn))
        print('\n init target_pos: ', self.target_pos)
        print('\n init target_rpy: ', self.target_rpy)
        

    def setRobotPosAngle(self, posX, posY, posZ, angleR, angleP, angleY):
        # step_size = 0.01  # 1cm
        # angle_step = math.radians(1)  # 1度转弧度
        if len(self.target_pos) < 3 or len(self.target_rpy) < 3:
            print("位置或角度错误")
            return
        self.target_pos[0] += (posX / 100)
        self.target_pos[1] += (posY / 100)
        self.target_pos[2] += (posZ / 100)
        self.target_rpy[0] += math.radians(angleR)
        self.target_rpy[1] += math.radians(angleP)
        self.target_rpy[2] += math.radians(angleY)
        # print(self.end_effector_link_index)
        # 计算逆运动学
        target_quat = p.getQuaternionFromEuler(self.target_rpy)
        joint_angles = p.calculateInverseKinematics(
            self.robotId,
            self.last_joint_index,
            self.target_pos,
            target_quat,
            maxNumIterations=100,
            residualThreshold=1e-5,
            jointDamping=[0.1] * 6
            # jointIndices=controlled_joints
        )
        print('\n move target_pos: ', self.target_pos)
        print('\n move target_rpy: ', self.target_rpy)
        print('joint_angles:', joint_angles)

        # 应用关节控制
        for i, joint_index in enumerate(self.controlled_joints):
            p.setJointMotorControl2(
                self.robotId,
                joint_index,
                p.POSITION_CONTROL,
                targetPosition=joint_angles[i],
                force=500
            )
        
        # 步进仿真
        p.stepSimulation()
        time.sleep(1./240.)

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
        slider_row = QHBoxLayout()  # 创建水平布局用于放置滑块
        control_layout.addLayout(slider_row)

        # 关节控制滑块
        for i in range(self.bullet_widget.num_joints):
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(0)
            slider.setPageStep(5)
            slider.setFixedWidth(100)  # 设置滑块宽度
            slider.valueChanged.connect(
                lambda value, idx=i: self.set_joint_angle(idx, value)
            )
            
            # 创建标签并设置字体大小
            if (i==0):
                joint_range = "-160°~160°"
            elif (i == 1):
                joint_range = "0°~210°"
            elif (i == 2):
                joint_range = "-176°~0°"
            elif (i == 3):
                joint_range = "-150°~150°"
            elif (i == 4):
                joint_range = "-90°~90°"
            elif (i == 5):
                joint_range = "-90°~90°"
            joint_label = QLabel(f"关节{i+1} : {joint_range}")
            joint_label.setStyleSheet("font-size: 16px;")  # 设置字体大小为16px

            slider_container = QVBoxLayout()  # 每个滑块单独一个垂直布局
            slider_container.addWidget(joint_label)
            slider_container.addWidget(slider)
            slider_row.addLayout(slider_container)
            slider_row.addSpacing(30)
            self.joint_sliders.append(slider)

        # 添加复位按钮
        reset_btn = QPushButton("复位")
        reset_btn.setFixedWidth(100)  # 增加按钮宽度
        reset_btn.setFixedHeight(50)  # 增加按钮高度
        reset_btn.setStyleSheet("font-size: 20px;")  # 设置字体大小
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
            # positionGain=0.5,  # 位置增益
            # velocityGain=1.0,  # 速度增益
            force=500
        )

    def reset_joints(self):
        # 重置相机参数初始化
        self.bullet_widget.camera_distance = 2.0
        self.bullet_widget.camera_yaw = 35
        self.bullet_widget.camera_pitch = -30
        self.bullet_widget.camera_target = np.array([-0.1, 0.2, 0.0])
        # 计算视图矩阵
        self.bullet_widget.update_frame()
        
        # 关节复位
        for slider in self.joint_sliders:
            slider.setValue(0)
        
        # 退出时复位关节位置
        for i, pos in zip(self.bullet_widget.controlled_joints, self.bullet_widget.initial_joint_positions):
            p.resetJointState(self.bullet_widget.robotId, i, pos)
            p.setJointMotorControl2(
                self.bullet_widget.robotId,
                i,
                p.POSITION_CONTROL,
                targetPosition=pos,
                force=500
            )

        # 最后步进一次确保复位
        p.stepSimulation()
        time.sleep(0.5)

    def setRobotPosAngle(self, posX, posY, posZ, angleR, angleP, angleY):
        print(posX, posY, posZ, angleR, angleP, angleY)
        self.bullet_widget.setRobotPosAngle(posX, posY, posZ, angleR, angleP, angleY)
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotWindow()
    window.show()
    sys.exit(app.exec_())