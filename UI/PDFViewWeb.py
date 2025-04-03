import sys, os
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebEngineWidgets import QWebEngineSettings
from PyQt5.QtCore import QUrl
from PyQt5 import QtCore

class PDFViewer(QMainWindow):
    def __init__(self, pdf_path):
        super().__init__()
        self.setWindowTitle("PDF Viewer")
        self.setMinimumSize(800, 600)

        # 使用 QWebEngineView 加载 PDF
        self.web_view = QWebEngineView()
        # 在初始化时启用 PDF 支持
        self.web_view.settings().setAttribute(QWebEngineSettings.PluginsEnabled, True)
        self.web_view.settings().setAttribute(QWebEngineSettings.PdfViewerEnabled, True)
        pdf_path = os.path.abspath(pdf_path)  # 转换为绝对路径
        self.web_view.setUrl(QUrl.fromLocalFile(pdf_path))
        self.setCentralWidget(self.web_view)

if __name__ == "__main__":
    QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    viewer = PDFViewer("test2.pdf")
    viewer.show()
    sys.exit(app.exec_())
