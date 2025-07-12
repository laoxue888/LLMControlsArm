#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from PySide6.QtCore import QObject, Signal, QThread
from PySide6.QtWidgets import QApplication
from NodeGraphQt import BaseNode, NodeBaseWidget
from utils.general import find_nodes_folder
from PySide6.QtGui import QImage
import numpy as np
from PIL import Image, ImageQt

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
__all__ = ('ScreenshotNode',)


def images_are_different(img1, img2):
    # 转换为numpy数组
    arr1 = np.array(img1)
    arr2 = np.array(img2)
    data_flag = np.array_equal(arr1, arr2)
    if data_flag:
        return False
    return True

def get_clipboard_content():
    app = QApplication.instance()
    clipboard = app.clipboard()

    # 检查剪贴板中是否有图片
    if clipboard.mimeData().hasImage():
        image = clipboard.image()
        if not image.isNull():
            # image.save("clipboard_image.png")
            # print("图片已保存为 clipboard_image.png")
            image = ImageQt.fromqimage(image)
            return True, image
    return False, None

    # # 获取文本内容
    # text = clipboard.text()
    # if text:
    #     print("剪贴板文本内容:", text)
    # else:
    #     print("剪贴板中没有文本或图片内容")

class ScreenshotNode(BaseNode, QObject):
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'Screenshot'

    def __init__(self):
        super().__init__()
        self.add_output('image')
        self.old_image = Image.new('RGBA', (1, 1))
    def execute(self):
        """节点执行函数"""
        # 这里加个循环，知道获取到图片为止
        while True:
            is_image, image = get_clipboard_content()
            if is_image:
                if self.old_image and images_are_different(self.old_image, image):
                    self.old_image = image
                    break
            QThread.msleep(50)
        setattr(self, 'image', image)

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

    def close_node(self,):
        """整个软件窗体关闭时调用"""

    def _del_node(self):
        """删除节点前调用"""
