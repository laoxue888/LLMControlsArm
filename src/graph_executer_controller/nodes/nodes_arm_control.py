#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from NodeGraphQt import BaseNode
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from panda_arm_msg.srv import ControlRvizArm, ControlRvizArm_Request
import rclpy
from openai import OpenAI
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import json
from rclpy.node import Node
import re

__all__ = ['PandaArmControlNode','PandaArmDeepSeekControlNode']


class PandaArmDeepSeekControlNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = 'nodes.arm.control'
    NODE_NAME = 'panda arm controlled by deepseek'

    def __init__(self):
        super(PandaArmDeepSeekControlNode, self).__init__()
        self.add_input('text_in')
        self.add_output('next_step')

        self.add_text_input('max_mem_len', label="Max Memory Length")
        self.set_property("max_mem_len", "20")
        
        self.srv_node = Node(self.NODE_NAME.replace(' ', '_'))
        self.cli = self.srv_node.create_client(ControlRvizArm, 'control_rviz_arm')

        self.client = OpenAI(api_key="your api key", base_url="https://api.deepseek.com")
        system_prompt = """
                        你是一个六自由度的机械臂，其中的answer答复要有拟人性。

                        请按照用户的意图给出机械臂末端的位置和姿态。

                        json输出示例:
                        {
                            "position.x": 0.3,
                            "position.y": 0.3,
                            "position.z": 0.2,
                            "orientation.x": 0.0,
                            "orientation.y": 0.0,
                            "orientation.z": 0.0,
                            "orientation.w": 1.0,
                            "gripper_state": "open", #只有open和close
                            "answer": "已给出目标位置和姿态，后续将进行规划执行动作。"
                        }
                        """
        system_message = {"role": "system", "content": system_prompt}

        self.messages = [system_message]

        self.sub_end_pose = self.srv_node.create_subscription(
            Float64MultiArray,
            '/end_effector_pose',
            self.end_pose_listener_callback,
            10)
        self.sub_end_pose  # prevent unused variable warning

        self.end_pose = None
        self.is_get_end_pose = False

    def end_pose_listener_callback(self, msg):
        """"""
        self.end_pose = msg
        # print(f"end_pose: {self.end_pose}")
        self.is_get_end_pose = True

    def execute(self):
        """"""
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.srv_node.get_logger().info('service not available, waiting again...')

        # 获取语音节点的文本输入
        text_in = self.input(0).connected_ports()[0].node().text_out
        
        # 通过话题获取当前的机械臂位置状态
        # position, rotation = self.get_end_effector_pose()
        while not self.is_get_end_pose:
            rclpy.spin_once(self.srv_node)
        end_pose = self.end_pose.data[:]
        # 重置标志位
        self.is_get_end_pose = False

        user_prompt = """
                        机械臂末端的当前位置和姿态是： "position.x": %s,"position.y": %s,"position.z": %s,"orientation.x": %s,"orientation.y": %s,"orientation.z": %s,"orientation.w": %s,
                        用户的意图是：%s
                    """ % (end_pose[0], end_pose[1], end_pose[2], end_pose[3], end_pose[4], end_pose[5], end_pose[6], text_in)

        self.messages.append({"role": "user", "content": user_prompt})

        self.messageSignal.emit(f'{self.NODE_NAME}的LLM输入： "role": "user", "content": {user_prompt}')

        # 创建聊天请求
        chat_completion = self.client.chat.completions.create(
            messages=self.messages, model="deepseek-chat", )
        assistant_message = chat_completion.choices[0].message.content

        pattern = r'\{([^{}]*)\}'
        matches = re.findall(pattern, assistant_message)
        assistant_message = '{' + matches[0] + '}'

        self.messages.append(chat_completion.choices[0].message)

        # 如果超出最长记录长度，删除第二个消息
        if len(self.messages) > int(self.get_property("max_mem_len")):
            del self.messages[1:3]

        # 加判断，解析返回的json数据
        data_json = json.loads(assistant_message)
        self.messageSignal.emit(f'{self.NODE_NAME}的LLM输出：{data_json}')
        # self.text_out = data_json['answer']
        # self.messageSignal.emit(assistant_message)

        # 解析返回信号，通过服务发送位置信号
        request = ControlRvizArm_Request()
        request.position = [float(data_json['position.x']), 
                            float(data_json['position.y']), 
                            float(data_json['position.z']),
                            float(data_json['orientation.x']),
                            float(data_json['orientation.y']),
                            float(data_json['orientation.z']),
                            float(data_json['orientation.w'])]
        request.open_or_close = data_json['gripper_state']
        
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self.srv_node, future)

        response = future.result()

        if response.success:
            self.messageSignal.emit(f'{request.position} execution was successful.\n')
            self.text_out = data_json['answer'] + "执行成功！"
        else:
            self.messageSignal.emit(f'{request.position} execution failed.\n')
            self.text_out = data_json['answer'] + "执行失败！"

        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal


class PandaArmControlNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = 'nodes.arm.control'
    NODE_NAME = 'panda arm control'

    def __init__(self):
        super(PandaArmControlNode, self).__init__()
        # self.add_input('text_in')
        self.add_output('next_step')
        
        self.srv_node = Node(self.NODE_NAME.replace(' ', '_'))
        self.cli = self.srv_node.create_client(ControlRvizArm, 'control_rviz_arm')

        self.add_text_input('position.x', 'position.x', text='0.3')
        self.add_text_input('position.y', 'position.y', text='0.3')
        self.add_text_input('position.z', 'position.z', text='0.3')
        self.add_text_input('orientation.x', 'orientation.x', text='0.0')
        self.add_text_input('orientation.y', 'orientation.y', text='0.0')
        self.add_text_input('orientation.z', 'orientation.z', text='0.0')
        self.add_text_input('orientation.w', 'orientation.w', text='0.0')

        self.add_combo_menu('gripper_state', 'gripper_state', items=['open', 'close'])

    def execute(self):
        """"""
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # 通过服务发送位置信号
        request = ControlRvizArm_Request()
        request.position = [float(self.get_property('position.x')), 
                            float(self.get_property('position.y')), 
                            float(self.get_property('position.z')),
                            float(self.get_property('orientation.x')),
                            float(self.get_property('orientation.y')),
                            float(self.get_property('orientation.z')),
                            float(self.get_property('orientation.w'))]
        request.open_or_close = self.get_property('gripper_state')

        text_in = self.input(0).connected_ports()[0].node().text_out
 
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self.srv_node, future)

        response = future.result()

        if response.success:
            self.messageSignal.emit(f'{request.position} execution was successful.')
            self.text_out = "执行成功！"
        else:
            self.messageSignal.emit(f'{request.position} execution failed.')
            self.text_out = "执行失败！"

        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

