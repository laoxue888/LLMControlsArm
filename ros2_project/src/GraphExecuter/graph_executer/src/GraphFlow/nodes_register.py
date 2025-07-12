#! /usr/bin/python3
# -*- coding: utf-8 -*-
from pathlib import Path
from Qt import QtCore
import os
from NodeGraphQt import (
    NodeGraph,
    PropertiesBinWidget
)
from threading import Thread

# import example nodes from the "nodes" sub-package
# from nodes import *
import nodes
import time
import traceback
import importlib
import sys
import pkgutil

# /root/ros2_docker_car_ws/src/robot_controller_graph/src/GraphFlow
BASE_PATH = Path(__file__).parent.resolve() 
nodes_module_path = os.path.join(os.path.dirname(os.path.dirname(Path(BASE_PATH).absolute())), "nodes")
sys.path.insert(0, str(nodes_module_path))

class GraphFlow(QtCore.QObject):

    def __init__(self,messageSignal, parent=None):
        """"""
        super(GraphFlow, self).__init__(parent)
        # create graph controller.
        self.graph = NodeGraph(parent=parent)
        self.graph_widget = self.graph.widget
        self.messageSignal = messageSignal

        # set up context menu for the node graph.
        hotkey_path = os.path.join(BASE_PATH, 'hotkeys', 'hotkeys.json')
        self.graph.set_context_menu_from_file(hotkey_path, 'graph')
        ######################################在这里注册节点#####################################
        # registered example nodes. Tab键可弹出 ##Backdrop节点可以用于注释
        self.register_nodes_recursive(nodes, self.graph, self.messageSignal)
        #######################################################################################
        # # auto layout nodes.
        self.graph.auto_layout_nodes()

        # fit nodes to the viewer.
        self.graph.clear_selection()
        self.graph.fit_to_selection()

        self.threads = {}

        self.timeer_check_thread = QtCore.QTimer()
        self.timeer_check_thread.timeout.connect(self.check_thread_status)
        self.timeer_check_thread.start(500)

        self.init_gui()

        # 是否调试
        self.is_debug = True
    def register_nodes_recursive(self, package, graph, messageSignal):
        # 获取包的路径
        package_path = Path(package.__path__[0])
        
        # 遍历包中的所有模块
        for (_, module_name, _) in pkgutil.walk_packages([str(package_path)], prefix=f"{package.__name__}."):
            try:
                module = importlib.import_module(module_name)
                if hasattr(module, '__all__'):
                    for node in module.__all__:
                        try:
                            graph.register_node(getattr(module, node))
                            messageSignal.emit(f'registered {node}')
                        except Exception as e:
                            if '.' in module.__name__ :
                                pass
                            else:
                                messageSignal.emit(f'{e} load failed')
            except ImportError as e:
                messageSignal.emit(f"Error importing {module_name}: {e}")

    def check_thread_status(self):
        """检查线程状态"""
        for node_name, thread in list(self.threads.items()):
            if not thread.is_alive():
                self.messageSignal.emit(f"Thread {node_name} has finished")
                del self.threads[node_name]

    def init_gui(self):
        """初始化Graph的GUI"""
        # 设置背景颜色
        # self.graph.set_background_color(255, 255, 255)

        # create a node properties bin widget.
        properties_bin = PropertiesBinWidget(node_graph=self.graph)
        properties_bin.setWindowFlags(QtCore.Qt.Tool)

        # example show the node properties bin widget when a node is double-clicked.
        def display_properties_bin(node):
            if not properties_bin.isVisible():
                properties_bin.show()

        # wire function to "node_double_clicked" signal.
        self.graph.node_double_clicked.connect(display_properties_bin)
        self.graph.nodes_deleted.connect(self.on_nodes_deleted)
        self.graph.node_created.connect(self.on_node_created)
    
    def on_node_created(self,node_obj):
        print(f"Node created: {node_obj}")
        # if hasattr(node_obj, 'set_widget_parent'):
        #     node_obj.set_widget_parent(self) # 运行节点函数

    def on_nodes_deleted(self, node_id):
        print(f"Node deleted: {node_id}")

    def stop_all_nodes(self):
        """停止所有节点的执行"""
        all_nodes = self.graph.all_nodes()
        for node in all_nodes:
            if hasattr(node, 'stop_execute'):
                 node.stop_execute()

    def execute_selected_nodes(self):
        """从选定节点开始执行下游节点"""
        selected_nodes = self.graph.selected_nodes()
        if not selected_nodes:
            self.messageSignal.emit("Please select a start node first")
            return

        start_node = selected_nodes[0]
        self.messageSignal.emit(f"=== from {start_node.name()} node to run ===")
        # 如果节点禁止用，则退出线程
        if start_node.disabled():
            return
        def run_graph(messageSignal):
            """"""
            if self.is_debug:
                self.get_execution_order(start_node)
            else:
                try:
                    self.get_execution_order(start_node)
                except Exception as err:
                    messageSignal.emit(f"Thread error: {err}")
                    messageSignal.emit(err.__traceback__.tb_frame.f_globals["__file__"])   # 发生异常所在的文件
                    messageSignal.emit(err.__traceback__.tb_lineno)                        # 发生异常所在的行数
                

        if start_node.NODE_NAME not in self.threads.keys():
            # 可以在这里创建多线程去执行，每一个初始节点创建一个线程
            thread = Thread(target=run_graph, name=start_node.NODE_NAME, args=(self.messageSignal,), daemon=True)
            thread.start()
            self.threads[start_node.NODE_NAME] = thread
        else:
            self.messageSignal.emit(f"Thread {start_node.NODE_NAME} is already running")

    def execute_all_nodes(self):
        """执行图中的所有节点（按依赖顺序）"""
        # self.stop_all_nodes()
        # time.sleep(1)  # 等待节点停止

        all_nodes = self.graph.all_nodes()

        # 找到所有没有输出的节点作为目标节点
        obj_nodes = [node for node in all_nodes if not any( #没有输出的就是目标节点
            port.connected_ports()
            for port in node.outputs().values()
        )]

        if not obj_nodes:
            self.messageSignal.emit("No executable object node is found in the diagram")
            return

        self.messageSignal.emit("=== Start executing the entire graph ===")

        def run_graph(obj_node, messageSignal):
            """"""
            if obj_node.disabled():
                return
            if self.is_debug:
                self.get_execution_order(obj_node)
            else:
                try:
                    self.get_execution_order(obj_node)
                except Exception as err:
                    messageSignal.emit(f"Thread error: {err}")
                    messageSignal.emit(err.__traceback__.tb_frame.f_globals["__file__"])   # 发生异常所在的文件
                    messageSignal.emit(err.__traceback__.tb_lasti)                        # 发生异常所在的行数


        # 可以在这里创建多线程去执行，每一个初始节点创建一个线程
        for obj_node in obj_nodes:
            if obj_node.NODE_NAME not in self.threads.keys():
                # 可以在这里创建多线程去执行，每一个初始节点创建一个线程
                thread = Thread(target=run_graph, name=obj_node.NODE_NAME, args=(obj_node, self.messageSignal,), daemon=True)
                thread.start()
                self.threads[obj_node.NODE_NAME] = thread
            else:
                self.messageSignal.emit(f"Thread {obj_node.NODE_NAME} is already running")


    def get_execution_order(self, obj_node):
        """获取从指定节点开始的下游节点执行顺序（拓扑排序）"""
        visited = set()
        # execution_order = []

        def visit_up(node):
            if node in visited:
                return

            # 首先处理所有上游节点
            for port in node.inputs().values():
                for connected_port in port.connected_ports():
                    visit_up(connected_port.node())

            visited.add(node)
            node.set_messageSignal(self.messageSignal)
            if hasattr(node, 'execute'):
                 node.execute() # 运行节点
        visit_up(obj_node)

    def save_session(self,):
        """
        Prompts a file save dialog to serialize a session if required.
        """
        current = self.graph.current_session()
        if current:
            self.graph.save_session(current)
            msg = 'Session layout saved:\n{}'.format(current)
            viewer = self.graph.viewer()
            viewer.message_dialog(msg, title='Session Saved')
        else:
            self.save_session_as()

    def save_session_as(self,):
        """
        Prompts a file save dialog to serialize a session.
        """
        current = self.graph.current_session()
        file_path = self.graph.save_dialog(current)

        def save_session(file_path):
            from pathlib import Path
            import json

            def convert_paths_to_strings(obj):
                if isinstance(obj, dict):
                    return {k: convert_paths_to_strings(v) for k, v in obj.items()}
                elif isinstance(obj, (list, tuple, set)):
                    return type(obj)(convert_paths_to_strings(x) for x in obj)
                elif isinstance(obj, Path):
                    return str(obj)
                return obj

            serialized_data = self.graph.serialize_session()
            serialized_data = convert_paths_to_strings(serialized_data)
            file_path = file_path.strip()

            def default(obj):
                if isinstance(obj, set):
                    return list(obj)
                return obj

            with open(file_path, 'w') as file_out:
                json.dump(
                    serialized_data,
                    file_out,
                    indent=2,
                    separators=(',', ':'),
                    default=default
                )

            # update the current session.
            self.graph._model.session = file_path

        if file_path:
            save_session(file_path)

    def close_event(self,):
        all_nodes = self.graph.all_nodes()
        for node in all_nodes:
            if hasattr(node, 'close_node'):
                 node.close_node() # 运行节点函数

