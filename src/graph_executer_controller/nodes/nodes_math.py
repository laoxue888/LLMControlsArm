#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from NodeGraphQt import BaseNode
import math

__all__ = ['NumberNode', 'AddNode', 'PrintNode']

class NumberNode(BaseNode):
    """数字节点，提供固定数值"""
    __identifier__ = 'nodes.math'
    NODE_NAME = 'Number float'

    def __init__(self):
        super(NumberNode, self).__init__()
        self.add_output('value')
        self.add_text_input('number', '', '1.0')

    def get_value(self):
        return float(self.get_property('number'))

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

class AddNode(BaseNode):
    """加法节点，执行两个数的加法"""
    __identifier__ = 'nodes.math'
    NODE_NAME = 'Add'

    def __init__(self):
        super(AddNode, self).__init__()
        self.add_input('input_1')
        self.add_input('input_2')
        self.add_output('output')

    def execute(self):
        # 获取输入值
        input1 = self.input(0)
        input2 = self.input(1)

        val1 = input1.connected_ports()[0].node().get_value() if input1.connected_ports() else 0
        val2 = input2.connected_ports()[0].node().get_value() if input2.connected_ports() else 0

        result = val1 + val2
        # print(f"{self.name()}: {val1} + {val2} = {result}")
        return result

    def get_value(self):
        return self.execute()

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

class PrintNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = 'nodes.math'
    NODE_NAME = 'Print'

    def __init__(self):
        super(PrintNode, self).__init__()
        self.add_input('input')
        self.value = None

    def execute(self):
        input_port = self.input(0)
        if input_port.connected_ports():
            self.value = input_port.connected_ports()[0].node().execute()
            self.messageSignal.emit(f"{self.name()} Output result: {self.value}")

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal
