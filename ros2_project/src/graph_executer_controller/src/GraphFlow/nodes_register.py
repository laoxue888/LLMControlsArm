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
from nodes import *
import nodes

# 然后正常使用 NodeGraphQt
BASE_PATH = Path(__file__).parent.resolve()

class GraphFlow:
    def __init__(self,messageSignal):
        """"""
        # create graph controller.
        self.graph = NodeGraph()
        self.graph_widget = self.graph.widget
        self.messageSignal = messageSignal

        # set up context menu for the node graph.
        hotkey_path = os.path.join(BASE_PATH, 'hotkeys', 'hotkeys.json')
        self.graph.set_context_menu_from_file(hotkey_path, 'graph')
        ######################################在这里注册节点#####################################
        # registered example nodes. Tab键可弹出 ##Backdrop节点可以用于注释
        for module in nodes.__all__:
            for node in eval(module).__all__:
                try:
                    self.graph.register_node(eval("{}.{}".format(str(module), str(node))))
                    self.messageSignal.emit(f'registered {node}')
                except Exception as e:
                    self.messageSignal.emit(f'{e} load failed')
        # for node in nodes_math.__all__:
        #     try:
        #         self.graph.register_node(eval("{}.{}".format('nodes_math', node)))
        #         self.messageSignal.emit(f'registered {node}')
        #     except Exception as e:
        #         self.messageSignal.emit(f'{e} load failed')
        # for node in nodes_read_data.__all__:
        #     try:
        #         self.graph.register_node(eval("{}.{}".format('nodes_read_data', node)))
        #         self.messageSignal.emit(f'registered {node}')
        #     except Exception as e:
        #         self.messageSignal.emit(f'{e} load failed')
        # for node in nodes_speech.__all__:
        #     try:
        #         self.graph.register_node(eval("{}.{}".format('nodes_speech', node)))
        #         self.messageSignal.emit(f'registered {node}')
        #     except Exception as e:
        #         self.messageSignal.emit(f'{e} load failed')
        #######################################################################################
        # # auto layout nodes.
        self.graph.auto_layout_nodes()

        # fit nodes to the viewer.
        self.graph.clear_selection()
        self.graph.fit_to_selection()

        self.init_gui()

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

        # 加载
        # try:
        #     self.graph.load_session(os.path.join(os.getcwd(), 'docs', 'examples_graph','example_graph.json'))
        # except Exception as err:
        #     print(err)

    def execute_downstream(self):
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
            try:
                self.get_execution_order(start_node)
            except Exception as err:
                messageSignal.emit(f"Thread error: {err}")

        # 可以在这里创建多线程去执行，每一个初始节点创建一个线程
        thread = Thread(target=run_graph, args=(self.messageSignal,), daemon=True)
        thread.start()
        # self.messageSignal.emit("=== 执行完成 ===\n")

    def execute_all_nodes(self):
        """执行图中的所有节点（按依赖顺序）"""
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
            try:
                self.get_execution_order(obj_node)
            except Exception as err:
                messageSignal.emit(f"Thread error: {err}")

        # 可以在这里创建多线程去执行，每一个初始节点创建一个线程
        for obj_node in obj_nodes:
            thread = Thread(target=run_graph, args=(obj_node, self.messageSignal), daemon=True)
            thread.start()
        # self.messageSignal.emit("=== 全部执行完成 ===\n")

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

