#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from PySide6.QtCore import QObject
from NodeGraphQt import BaseNode, NodeBaseWidget
from utils.general import find_nodes_folder

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
__all__ = ('NameNode')

class NameNode(BaseNode, QObject):
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'name'

    def __init__(self):
        super(NameNode, self).__init__()
        self.add_input('image_data')
        self.add_output('processed_image_data')

    def execute(self):
        """节点执行函数"""
        image_data = getattr(self.input(0).connected_ports()[0].node(), self.input(0).connected_ports()[0].name())

        # 处理数据
        processed_image_data = image_data

        # 输出数据
        setattr(self, 'processed_image_data', processed_image_data)


    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

    def set_widget_parent(self, parent):
        self.setParent(parent)

    def close_node(self,):
        """整个软件窗体关闭时调用"""
        # self.myui.close()
        # del self.myui
        # self.video_thread.quit()  # 一定要在这里释放线程
        # self.video_thread.wait()

    def _del_node(self):
        """删除节点前调用"""
        # self.myui.close()
        # del self.myui
        # self.video_thread.quit()  # 一定要在这里释放线程
        # self.video_thread.wait()
