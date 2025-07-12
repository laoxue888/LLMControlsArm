#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from PySide6.QtCore import QObject
from NodeGraphQt import BaseNode, NodeBaseWidget
from utils.general import find_nodes_folder
from pix2tex.cli import LatexOCR
from PySide6.QtCore import *
from PySide6.QtWidgets import *
from PySide6.QtGui import *
import PIL
import numpy as np
import time
import os
from io import BytesIO
from PIL import Image
import numpy as np
import matplotlib.font_manager as mfm
from matplotlib import mathtext

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
__all__ = ('Pix2TextNode', 'ImageDisplayNode')

def latex2img(text, size=32, color=(0.1,0.1,0.1), out=None, **kwds):
    """LaTex数学公式转图片
        text        - 文本字符串，其中数学公式须包含在两个$符号之间
        size        - 字号，整型，默认32
        color       - 颜色，浮点型三元组，值域范围[0,1]，默认深黑色
        out         - 文件名，仅支持后缀名为.png的文件名。若为None，则返回PIL图像对象
        kwds        - 关键字参数
                        dpi         - 输出分辨率（每英寸像素数），默认72
                        family      - 系统支持的字体，None表示当前默认的字体
                        weight      - 笔画轻重，可选项包括：normal（默认）、light和bold
        """

    assert out is None or os.path.splitext(out)[1].lower() == '.png', '仅支持后缀名为.png的文件名'

    for key in kwds:
        if key not in ['dpi', 'family', 'weight']:
            raise KeyError('不支持的关键字参数：%s'%key)

    dpi = kwds.get('dpi', 72)
    family = kwds.get('family', None)
    weight = kwds.get('weight', 'normal')

    bfo = BytesIO() # 创建二进制的类文件对象
    prop = mfm.FontProperties(family=family, size=size, weight=weight)
    mathtext.math_to_image(text, bfo, prop=prop, dpi=dpi)
    im = Image.open(bfo)

    r, g, b, a = im.split()
    r, g, b = 255-np.array(r), 255-np.array(g), 255-np.array(b)
    a = r/3 + g/3 + b/3
    r, g, b = r*color[0], g*color[1], b*color[2]

    im = np.dstack((r,g,b,a)).astype(np.uint8)
    im = Image.fromarray(im)

    if out is None:
        return im
    else:
        im.save(out)
        print('生成的图片已保存为%s'%out)

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

    def camera_callback(self, data):
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


class ImageDisplayWidget(QWidget):
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
    NODE_NAME = 'Formula display'

    def __init__(self):
        super(ImageDisplayNode, self).__init__()
        # self.add_input('image_data')
        self.add_input('formula_latex_data')
        # self.add_output('image_data_out')
        self.add_output('spin_once')

        self.add_checkbox("open_window", text='show window')
        self.myui = ImageDisplayWidget()
        self.myui.set_node_obj(self)
        self.chk_value_changed()

        window_widget = self.get_widget("open_window")
        window_widget.value_changed.connect(self.chk_value_changed)

        # 创建工作线程
        self.video_thread = VideoThread(self.myui)
        # self.video_thread.setParent(self.myui)
        self.video_thread.start()
        self.video_thread.update_image.connect(self.myui.label_img.setPixmap)

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.myui.update)
        self.update_timer.start(500)

    def execute(self):
        """节点执行函数"""
        formula_latex_data = getattr(self.input(0).connected_ports()[0].node(), self.input(0).connected_ports()[0].name())
        formula_latex_data = '$' + fr"{formula_latex_data}" + '$'
        image_data = latex2img(formula_latex_data, size=48, color=(0.9, 0.1, 0.1))
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

    def close_node(self, ):
        """整个软件窗体关闭时调用"""
        self.myui.close()
        del self.myui
        self.video_thread.quit()  # 一定要在这里释放线程
        self.video_thread.wait()

    def _del_node(self):
        """删除节点前调用"""
        self.myui.close()
        del self.myui
        self.video_thread.quit()
        self.video_thread.wait()


class Pix2TextNode(BaseNode, QObject):
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'pix2text'

    def __init__(self):
        super(Pix2TextNode, self).__init__()
        self.add_input('image')
        self.add_output('text')
        self.add_output('spin_once')
        self.p2t = LatexOCR()

    def execute(self):
        """节点执行函数"""
        img_fp = getattr(self.input(0).connected_ports()[0].node(), self.input(0).connected_ports()[0].name())
        text = self.p2t(img_fp)
        self.messageSignal.emit(text)
        # 输出数据
        setattr(self, 'text', text)

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

    def set_widget_parent(self, parent):
        """"""

    def close_node(self,):
        """整个软件窗体关闭时调用"""

    def _del_node(self):
        """删除节点前调用"""
