#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from NodeGraphQt import BaseNode, NodeBaseWidget
from Qt import QtCore, QtWidgets
from PySide6.QtCore import *
from PySide6.QtWidgets import *
from PySide6.QtGui import *
import numpy as np
import os, requests
from utils.general import find_nodes_folder
import cv2
from utils.general import get_execution_order
import time
import PIL

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
__all__ = ('CameraDataNode', 'ImageDisplayNode', 'SpinWorkflow')

class SpinWorkflow(BaseNode, QObject):
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'spin workflow'

    def __init__(self):
        super(SpinWorkflow, self).__init__()
        self.add_input('spin_once')
        self.add_checkbox('is_display_loop', text='is_display_loop')

    def execute(self):
        """节点执行函数"""
        if not self.get_property("is_display_loop"):
            return
        else:
            execution_order = get_execution_order(self)[:-1]
            while self.get_property("is_display_loop"):
                for node in execution_order:
                    if hasattr(node, 'execute'):
                        node.execute() # 运行节点

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

    def close_node(self,):
        """整个软件窗体关闭时调用"""

    def _del_node(self):
        """删除节点前调用"""

class CameraDataNode(BaseNode, QObject):
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'get camera data'

    def __init__(self):
        super(CameraDataNode, self).__init__()
        self.add_output('image_data')
        self.add_checkbox("is_lock_cam", text='is_lock_cam')
        window_widget = self.get_widget("is_lock_cam")
        window_widget.value_changed.connect(self.release_cam)
        self.cam = None

    def release_cam(self):
        """"""
        if self.cam is not None:
            self.cam.release()

    def execute(self):
        """节点执行函数"""
        if (not self.cam) and self.get_property("is_lock_cam"):
            # 打开摄像头（0通常是默认摄像头）
            self.cam = cv2.VideoCapture(0)
            # 检查摄像头是否成功打开
            if not self.cam.isOpened():
                print("无法打开摄像头")
                exit()
        if not self.get_property("is_lock_cam"):
            # 打开摄像头（0通常是默认摄像头）
            self.cam = cv2.VideoCapture(0)
            # 检查摄像头是否成功打开
            if not self.cam.isOpened():
                print("无法打开摄像头")
                exit()

        # 读取一帧
        ret, frame = self.cam.read()# mdarray: 480, 640, 3
        if ret:
            setattr(self, 'image_data', frame)
        else:
            setattr(self, 'image_data', None)

        if self.get_property("is_lock_cam"):
            pass
        else:
            self.cam.release()
            self.cam = None


    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

    def close_node(self,):
        """整个软件窗体关闭时调用"""

    def _del_node(self):
        """删除节点前调用"""

class VideoThread(QThread, QObject):
    # 定义一个信号，用于发送处理后的数据
    update_image = Signal(QPixmap)
    
    def __init__(self, widget_window=None):
        super().__init__()
        self.widget_window = widget_window

        self.num_frame = 0
        self.num_time = 0.0
        self.before_time = None
        self.fps = 0.0

    def camera_callback(self,data):
        """显示图像的函数，要使用信号来调用，不可外部调用"""
        if isinstance(data, PIL.Image.Image):
            data = np.array(data)
        image_array = data
        # print(image_array.shape) # (480, 640, 3)

        height, width, channel = image_array.shape
        bytesPerLine = 3 * width
        qimage = QImage(image_array.data, width, height, bytesPerLine, QImage.Format.Format_BGR888)
        pixmap = QPixmap.fromImage(qimage)

        painter = QPainter(pixmap)
        pen = QPen()
        pen.setWidth(3)
        pen.setColor(Qt.GlobalColor.red)
        painter.setPen(pen)
        if self.before_time:
            self.num_frame += 1
            self.num_time += time.time() - self.before_time
            if self.num_frame % 10 == 0:
                self.fps = self.num_frame / self.num_time
                # print(f"fps: {fps:.2f}")
                self.num_time = 0.0
                self.num_frame = 0
        self.before_time = time.time()
        painter.drawText(595, 460, 60, 30, Qt.AlignmentFlag.AlignLeft, f"fps: {round(self.fps)}")
        painter.end()

        self.update_image.emit(pixmap.scaled(self.widget_window.label_img.size(), Qt.KeepAspectRatio))

class ImageDisplayWidget(QtWidgets.QWidget):
    """
    Custom widget to be embedded inside a node.
    """
    def __init__(self, parent=None):
        super(ImageDisplayWidget, self).__init__(parent)
        from ui.nodes.nodes_display import ui_image_display

        self.ui = ui_image_display.Ui_ImageDisplayForm()
        self.ui.setupUi(self)
        # logging.getLogger().setLevel(logging.WARNING)
        self.label_img = QLabel(self)
        self.label_img.setMouseTracking(True)
        self.ui.verticalLayout.addWidget(self.label_img)

    def set_node_obj(self, obj):
        """"""
        self.node_obj = obj

    def closeEvent(self, event):
        self.node_obj.set_property("open_window", False)
        return super().closeEvent(event)

class ImageDisplayNode(BaseNode, QObject):
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'Image display'

    def __init__(self):
        super(ImageDisplayNode, self).__init__()
        self.add_input('image_data')
        # self.add_input('formula_latex_data')
        self.add_output('image_data_out')
        self.add_output('spin_once')

        self.add_checkbox("open_window", text='show window')
        self.myui=ImageDisplayWidget()
        self.myui.set_node_obj(self)
        self.chk_value_changed()

        window_widget = self.get_widget("open_window")
        window_widget.value_changed.connect(self.chk_value_changed)

        # 创建工作线程
        self.video_thread = VideoThread(self.myui)
        # self.video_thread.setParent(self.myui)
        self.video_thread.start()
        self.video_thread.update_image.connect(self.myui.label_img.setPixmap)

        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.myui.update)
        self.update_timer.start(500)

    def execute(self):
        """节点执行函数"""
        # image_data = self.input(0).connected_ports()[0].node().processed_image_data
        image_data = getattr(self.input(0).connected_ports()[0].node(), self.input(0).connected_ports()[0].name())
        # formula_latex_data = getattr(self.input(1).connected_ports()[0].node(), self.input(1).connected_ports()[0].name())
        setattr(self, 'image_data_out', image_data)
        if image_data is not None:
            self.video_thread.camera_callback(image_data)

    def chk_value_changed(self):
        if self.get_property("open_window"):
            self.myui.show() 
        else:
            self.myui.close()

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal
    def set_widget_parent(self, parent):
        self.myui.setParent(parent)

    def close_node(self,):
        """整个软件窗体关闭时调用"""
        self.myui.close()
        del self.myui
        self.video_thread.quit() # 一定要在这里释放线程
        self.video_thread.wait()

    def _del_node(self):
        """删除节点前调用"""
        self.myui.close()
        del self.myui
        self.video_thread.quit()
        self.video_thread.wait()

    



