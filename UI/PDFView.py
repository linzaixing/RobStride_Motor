import sys
import fitz  # PyMuPDF
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QScrollArea,
    QVBoxLayout, QWidget, QPushButton, QHBoxLayout
)
from PyQt5 import QtCore
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt


class PDFViewer(QMainWindow):
    def __init__(self, file_path):
        super().__init__()
        self.setWindowTitle("PDF Viewer")
        # self.setGeometry(100, 100, 1000, 800)
        self.setMinimumSize(800, 600)

        # PDF文档变量
        self.doc = None
        self.zoom_factor = 1.5  # 默认缩放系数
        self.dpi_scale = 1.0  # 存储系统DPI缩放因子

        # 初始化UI
        self.init_ui()

        self.load_full_pdf(file_path)

    def init_ui(self):
        """初始化界面"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # 工具栏
        toolbar = QHBoxLayout()

        zoom_in_btn = QPushButton("放大(+)")
        zoom_in_btn.clicked.connect(self.zoom_in)
        toolbar.addStretch()
        toolbar.addWidget(zoom_in_btn)

        zoom_out_btn = QPushButton("缩小(-)")
        zoom_out_btn.clicked.connect(self.zoom_out)
        toolbar.addWidget(zoom_out_btn)
        toolbar.addStretch()

        # 滚动区域设置
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)

        # 创建一个容器Widget用于居中显示
        self.container = QWidget()
        self.container_layout = QHBoxLayout(self.container)
        self.container_layout.setContentsMargins(0, 0, 0, 0)

        # 用于显示PDF的标签（现在作为容器子部件）
        self.pdf_label = QLabel()
        self.pdf_label.setAlignment(Qt.AlignTop | Qt.AlignHCenter)  # 顶部对齐+水平居中

        # 添加左右弹性空间实现居中
        self.container_layout.addStretch()
        self.container_layout.addWidget(self.pdf_label)
        self.container_layout.addStretch()

        self.scroll_area.setWidget(self.container)
        main_layout.addWidget(self.scroll_area)
        main_layout.addLayout(toolbar)

    def load_full_pdf(self, file_path):
        """加载并显示整个PDF（保持水平居中）"""
        try:
            self.doc = fitz.open(file_path)

            # 获取系统DPI缩放因子
            screen = QApplication.primaryScreen()
            self.dpi_scale = screen.logicalDotsPerInch() / 96.0

            # 计算总高度和最大宽度
            total_height = 0
            max_width = 0
            pixmaps = []

            # 第一遍：计算尺寸
            for page in self.doc:
                mat = fitz.Matrix(self.zoom_factor*self.dpi_scale, self.zoom_factor*self.dpi_scale)
                pix = page.get_pixmap(matrix=mat,alpha=False,colorspace=fitz.csRGB)
                pixmaps.append(pix)
                total_height += pix.height
                max_width = max(max_width, pix.width)

            # 创建空白图像
            combined_img = np.zeros((total_height, max_width, 3), dtype=np.uint8)

            # 第二遍：拼接图像（自动居中）
            y_offset = 0
            for pix in pixmaps:
                # 计算水平居中位置
                x_offset = (max_width - pix.width) // 2
                img_array = np.frombuffer(pix.samples, dtype=np.uint8).reshape(
                    pix.height, pix.width, 3)
                combined_img[y_offset:y_offset + pix.height, x_offset:x_offset + pix.width] = img_array
                y_offset += pix.height

            # 转换为QImage
            height, width, _ = combined_img.shape
            q_img = QImage(
                combined_img.data,
                width,
                height,
                3 * width,
                QImage.Format_RGB888
            ).copy()

            # 显示图像
            self.pdf_label.setPixmap(QPixmap.fromImage(q_img))
            self.setWindowTitle(f"PDF Viewer - {file_path}")

        except Exception as e:
            self.pdf_label.setText(f"加载错误: {str(e)}")

    def zoom_in(self):
        """放大"""
        self.zoom_factor *= 1.2
        if self.doc:
            self.load_full_pdf(self.doc.name)

    def zoom_out(self):
        """缩小"""
        self.zoom_factor /= 1.2
        if self.zoom_factor < 0.5:
            self.zoom_factor = 0.5
        if self.doc:
            self.load_full_pdf(self.doc.name)


if __name__ == "__main__":
    QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    viewer = PDFViewer('test2.pdf')
    viewer.show()
    sys.exit(app.exec_())