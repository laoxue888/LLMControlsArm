
# --------------------------------------------------------------------
# -- Python Script File
# -- Created on 12/14/2024 15:45:12
# -- Author: xsf
# -- Comment: .
# --------------------------------------------------------------------
import sys, os, time, threading
from src.mainwindow import MainWindow
from PySide6.QtWidgets import QApplication, QMessageBox
from PySide6.QtWidgets import QApplication, QStyleFactory
from PySide6.QtGui import QFont, QPalette, QColor
import rclpy

def style(qApp):
    qApp.setStyleSheet(
        # reduce separator width
        "QMainWindow::separator {width: 1px} "
        # adjust menu style
        "QMenu::separator {height: 1px; margin-left: 6px; margin-right: 6px; background: rgba(155, 155, 155, 255);}"
        # set message box style
        "QMessageBox {color: white;}QMessageBox QLabel {color: yellow;}"
    )
    # # set style and color palette
    qApp.setStyle(QStyleFactory.create("Fusion"))
    # dark = QPalette()
    # dark.setColor(QPalette.ColorRole.Text, QColor(255, 255, 255))
    # dark.setColor(QPalette.ColorRole.WindowText, QColor(255, 255, 255))
    # dark.setColor(QPalette.ColorRole.Window, QColor(50, 50, 50))
    # dark.setColor(QPalette.ColorRole.Button, QColor(50, 50, 50))
    # dark.setColor(QPalette.ColorRole.Base, QColor(25, 25, 25))
    # dark.setColor(QPalette.ColorRole.AlternateBase, QColor(50, 50, 50))
    # dark.setColor(QPalette.ColorRole.ToolTipBase, QColor(200, 200, 200))
    # dark.setColor(QPalette.ColorRole.ToolTipText, QColor(50, 50, 50))
    # dark.setColor(QPalette.ColorRole.ButtonText, QColor(255, 255, 255))
    # dark.setColor(QPalette.ColorRole.BrightText, QColor(255, 50, 50))
    # dark.setColor(QPalette.ColorRole.Link, QColor(40, 130, 220))
    # dark.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, QColor(99, 99, 99))
    # dark.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, QColor(99, 99, 99))
    # dark.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, QColor(99, 99, 99))
    # dark.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Highlight, QColor(80, 80, 80))
    # dark.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.HighlightedText, QColor(99, 99, 99))
    # qApp.setPalette(dark)
    # //adjust font size
    font = QFont()
    # font.setFamily(font.defaultFamily())
    font.setPointSize(10)
    qApp.setFont(font)

# inner exception handler for the application
# def handle_exception(exc_type, exc_value, exc_traceback):
#     """异常处理函数"""
#     message = f"An exception of type {exc_type.__name__} occurred.\n{exc_value}"
#     QMessageBox.critical(None, "程序内部报错提示！", message)
#
#     logPath = os.path.join(os.getcwd(), "logs")
#     if not os.path.isdir(logPath):
#         os.makedirs(logPath)
#     logPath = os.path.join(logPath, f"{time.strftime('%Y-%m-%d', time.localtime())}.txt")
#
#     with open(logPath, 'a') as f:
#         saveTime = '[' + time.strftime('%H:%M:%S', time.localtime()) + ']: '
#         f.write(saveTime + message + '\n')
#
# sys.excepthook = handle_exception

if __name__ == '__main__':
    rclpy.init(args=None)
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    style(app)
    mainwindow = MainWindow()
    mainwindow.show()

    app.exec()


