from Qt import QtCore, QtWidgets
from NodeGraphQt import BaseNode, NodeBaseWidget
from ui.nodes.nodes_read_data.read_data_ui import Ui_ReadDataForm
import os
from pathlib import Path
from utils.general import *
import time, copy

__all__=['OpenDirectoryNode']

BASE_PATH = Path(__file__).parent.resolve()

class ReadDataForm(QtWidgets.QWidget):
    def __init__(self,):
        super().__init__()
        self.ui = Ui_ReadDataForm()
        self.ui.setupUi(self)

class MyCustomWidget(QtWidgets.QWidget):
    """
    Custom widget to be embedded inside a node.
    """
    def __init__(self, parent=None):
        super(MyCustomWidget, self).__init__(parent)
        self.read_data_form = ReadDataForm()
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.read_data_form)

        self.read_data_form.ui.pushButton_open_folder.clicked.connect(self.open_directory)

    def set_node_obj(self, obj):
        """"""
        self.node_obj = obj

    def open_directory(self,):
        """"""
        if self.node_obj.get_property('folder_path').strip():
            folder_path = getFoldersPath(self.node_obj.get_property('folder_path').strip())
        else:
            folder_path = getFoldersPath('/')
        if folder_path:
            self.node_obj.set_property('folder_path', folder_path[0])

class NodeWidgetWrapper(NodeBaseWidget):
    """
    Wrapper that allows the widget to be added in a node object.
    """
    def __init__(self, parent=None):
        super(NodeWidgetWrapper, self).__init__(parent)
        # set the name for node property.
        self.set_name('my_widget')
        # set the custom widget.
        self.set_custom_widget(MyCustomWidget())
        # connect up the signals & slots.
        self.wire_signals()
    def wire_signals(self):
        """"""
    def get_value(self):
        """"""
    def set_value(self, value):
        """"""

class OpenDirectoryNode(BaseNode):
    """
    Example node.
    """
    # set a unique node identifier.
    __identifier__ = find_nodes_folder(__file__)[1]
    # set the initial default node name.
    NODE_NAME = 'Open directory'
    def __init__(self):
        super(OpenDirectoryNode, self).__init__()

        # create input and output port.
        # self.add_input('')
        self.add_output('output')

        # add custom widget to node with "node.view" as the parent.
        node_widget = NodeWidgetWrapper(self.view)
        node_widget.get_custom_widget().set_node_obj(self)
        self.add_custom_widget(node_widget, tab='Custom')
        # self.set_icon(os.path.join(BASE_PATH, 'icon', 'star.png'))
        self.add_text_input('folder_path', '', tab='widgets')

    def execute(self, ):
        """graph管理器只执行这个函数"""
        folder_path = self.get_property('folder_path').strip()

        # print(self.properties().keys())
        self.messageSignal.emit(f'{self.NODE_NAME} executed.')
        return True

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

    def __del__(self):
        pass




