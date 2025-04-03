import sys
import numpy as np
import pybullet as p
import pybullet_data
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, 
    QVBoxLayout, QHBoxLayout, QSlider, 
    QLabel, QPushButton
)

class BulletWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(640, 480)
        # self.setAttribute(Qt.WA_OpaquePaintEvent)
        # self.setAttribute(Qt.WA_PaintOnScreen)  # Windows系统需要此属性
        
        # 初始化UI
        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignCenter)
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)
        
        # OpenGL/PyBullet初始化状态
        self.gl_initialized = False
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(33)  # 30 FPS

    def initBullet(self):
        if not self.gl_initialized:
            # 连接到当前OpenGL上下文
            self.physicsClient = p.connect(p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            self.planeId = p.loadURDF("./urdf/plane.urdf")
            try:
                self.robotId = p.loadURDF(r"./urdf/model.urdf", [0,0,0], useFixedBase=1)
            except:
                raise ValueError("URDF加载失败")
            self.gl_initialized = True

    def update_frame(self):
        if not self.gl_initialized:
            self.initBullet()
        
        p.stepSimulation()
        
        width = self.width()
        height = self.height()
        
        # 计算相机矩阵
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            [0, 0, 0.5], 2.5, 45, -30, 0, 2
        )
        proj_matrix = p.computeProjectionMatrixFOV(60, width/height, 0.1, 100)
        
        # 获取图像数据
        renderer = p.ER_TINY_RENDERER if p.DIRECT else p.ER_BULLET_HARDWARE_OPENGL
        _, _, rgb, _, _ = p.getCameraImage(
            width, height, view_matrix, proj_matrix,
            renderer=renderer
        )
        
        # 处理图像
        if rgb is not None:
            try:
                rgb_array = np.reshape(rgb, (height, width, 4))[:, :, :3]
                rgb_array = np.ascontiguousarray(rgb_array[..., [2,1,0]])  # BGR->RGB
                self.q_img = QImage(
                    rgb_array.data, width, height, 
                    3*width, QImage.Format_RGB888
                ).copy()  # 关键：复制数据避免内存释放
                self.label.setPixmap(QPixmap.fromImage(self.q_img))
            except Exception as e:
                print(f"图像处理失败: {e}")

# MainWindow类保持不变（与之前代码一致）

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("机械臂控制平台")
        self.setGeometry(100, 100, 1280, 720)
        
        # 主布局
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout(central_widget)
        
        # 3D显示区域
        self.bullet_widget = BulletWidget()
        layout.addWidget(self.bullet_widget, 3)
        
        # 控制面板
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        layout.addWidget(control_panel, 1)
        
        # 关节控制滑块
        self.joint_sliders = []
        for i in range(7):
            slider = QSlider(Qt.Vertical)
            slider.setRange(-180, 180)
            slider.setValue(0)
            slider.setPageStep(5)
            slider.valueChanged.connect(
                lambda value, idx=i: self.set_joint_angle(idx, value)
            )
            
            control_layout.addWidget(QLabel(f"关节 {i+1}"))
            control_layout.addWidget(slider)
            self.joint_sliders.append(slider)
        
        # 添加复位按钮
        reset_btn = QPushButton("复位")
        reset_btn.clicked.connect(self.reset_joints)
        control_layout.addWidget(reset_btn)

    def set_joint_angle(self, joint_idx, angle):
        p.setJointMotorControl2(
            self.bullet_widget.robotId,
            joint_idx,
            p.POSITION_CONTROL,
            targetPosition=np.deg2rad(angle),
            force=500
        )

    def reset_joints(self):
        for slider in self.joint_sliders:
            slider.setValue(0)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())