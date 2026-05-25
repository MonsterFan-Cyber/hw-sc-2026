"""Polygon Packing GUI - 入口点"""

import sys
import os
import logging

# 确保项目根目录在 Python 路径中
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

from gui.ui.main_window import MainWindow


def setup_logging():
    """配置日志"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S'
    )


def main():
    setup_logging()
    logger = logging.getLogger(__name__)
    logger.info("Starting Polygon Packing GUI")
    
    # High DPI 支持
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"  # 自动缩放
    os.environ["QT_SCALE_FACTOR_ROUNDING_POLICY"] = "PassThrough"  # 精确缩放，避免整数化导致的异常
    
    app = QApplication(sys.argv)
    app.setApplicationName("Polygon Packing GUI")
    app.setOrganizationName("Poly2")
    
    # 设置全局字体为微软雅黑，开启抗锯齿
    font = QFont("Microsoft YaHei", 9)
    font.setStyleStrategy(QFont.PreferAntialias)
    app.setFont(font)
    
    # 暗色主题（可选）
    # app.setStyle('Fusion')
    
    window = MainWindow()
    window.show()
    
    logger.info("Main window shown")
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()