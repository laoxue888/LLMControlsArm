#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from NodeGraphQt import BaseNode, NodeBaseWidget
from Qt import QtCore, QtWidgets
from PySide6.QtCore import *
from PySide6.QtWidgets import *
from PySide6.QtGui import *
import numpy as np
from ultralytics import YOLO
import os, requests
from utils.general import find_nodes_folder

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# __all__ = ['YOLOV13Node']

class YOLOV13Node(BaseNode, QObject):
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'yolov13'

    def __init__(self):
        super(YOLOV13Node, self).__init__()
        self.add_input('image_data')
        self.add_output('processed_image_data')
        self.yolo13_model = YOLO(os.path.join(BASE_DIR, 'pt_files', 'yolov13s.pt'))

    def execute(self):
        """节点执行函数"""
        image_data = getattr(self.input(0).connected_ports()[0].node(), self.input(0).connected_ports()[0].name())
        results = self.yolo13_model.predict(image_data, verbose=False, conf=0.5)
        processed_image_data = results[0].plot(conf=True,labels=True)
        # self.processed_image_data = processed_image_data
        setattr(self, 'processed_image_data', processed_image_data)

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

    def close_node(self,):
        """整个软件窗体关闭时调用"""

    def _del_node(self):
        """删除节点前调用"""
