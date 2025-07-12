#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from NodeGraphQt import BaseNode
import speech_recognition as sr
import pyttsx3, os
from vosk import Model, KaldiRecognizer, SetLogLevel
import pyaudio
import json
import numpy as np
from utils.general import download_and_extract_zip
import wave
import edge_tts
import asyncio
import tempfile
from playsound import playsound
from utils.general import get_execution_order
from utils.general import find_nodes_folder

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

__all__ = ['WhisperRecognitionNode', 'Pyttsx3SpeakNode', 'VOSKRecognitionNode', 'EdgeTTSSpeakNode', 'TextInputNode']


class TextInputNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'Text input'

    def __init__(self):
        super(TextInputNode, self).__init__()
        self.add_text_input('text_in')
        self.add_output('text_out')
        self.text_out = ""

    def execute(self):
        self.text_out = self.get_property('text_in')
        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal


class EdgeTTSSpeakNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'Speak by edge-tts '

    def __init__(self):
        super(EdgeTTSSpeakNode, self).__init__()
        # 初始化语音引擎
        self.engine = pyttsx3.init()
        # 设置语音属性（可选）
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', voices[0].id)  # 0通常是英文男声，1可能是英文女声，中文可能需要其他设置
        self.engine.setProperty('rate', 150)  # 语速
        self.add_input('text_in')

    def execute(self):
        """"""
        text = self.input(0).connected_ports()[0].node().text_out
        self.messageSignal.emit(f"{self.name()} Output result: {text}")
        asyncio.run(self.play_with_playsound(text))

        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    async def play_with_playsound(self,text):
        voice = 'zh-CN-XiaoxiaoNeural'
        # text = "你好，这是一个使用playsound播放的测试。"

        # 创建临时文件
        with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as tmp_file:
            tmp_path = tmp_file.name

        # 保存到临时文件
        communicate = edge_tts.Communicate(text, voice)
        await communicate.save(tmp_path)

        # 播放
        playsound(tmp_path)

        # 删除临时文件
        os.unlink(tmp_path)


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

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal


class VOSKRecognitionNode(BaseNode):
    """科学上网下载速度才快，否则只有几十kb/s"""
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'Speech recognition by vosk'

    def __init__(self):
        super(VOSKRecognitionNode, self).__init__()
        # 加载模型
        # https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip         41.87 M
        # https://alphacephei.com/vosk/models/vosk-model-cn-0.15.zip                1.67 G
        models_dir = os.path.join(BASE_DIR, 'res', 'models', 'VOSK')
        if not os.path.exists(models_dir):
            os.makedirs(models_dir)

        model_dir = os.path.join(BASE_DIR, 'res', 'models', 'VOSK', 'vosk-model-cn-0.15')
        if os.path.exists(model_dir):
            self.vosk_model = Model(model_dir)
        else:
            download_and_extract_zip('https://alphacephei.com/vosk/models/vosk-model-cn-0.15.zip',os.path.dirname(model_dir), os.path.dirname(model_dir))
            try:
                os.remove(os.path.join(os.path.dirname(model_dir), 'vosk-model-cn-0.15.zip'))
            except OSError:
                pass
            self.vosk_model = Model(model_dir)
        SetLogLevel(-1)

        self.add_output('text_out')
        self.text_out = ""

    def execute(self):
        while True:
            text = self.save_wave(self.vosk_model)
            if text != "" and text != None:
                self.messageSignal.emit(f"{self.name()} Output result: {text}")
                self.text_out = text
                break
        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    def save_wave(self, model):
        # 设置音频参数
        FORMAT = pyaudio.paInt16  # 音频流的格式
        RATE = 44100  # 采样率，单位Hz
        CHUNK = 4000  # 单位帧
        THRESHOLDNUM = 30  # 静默时间，超过这个个数就保存文件
        THRESHOLD = 50  # 设定停止采集阈值

        audio = pyaudio.PyAudio()
        stream = audio.open(format=FORMAT,
                            channels=1,
                            rate=RATE,
                            input=True,
                            frames_per_buffer=CHUNK)
        frames = []
        # print("开始录音...")
        self.messageSignal.emit(f"{self.name()} 开始录音...")
        count = 0
        while count < THRESHOLDNUM:
            data = stream.read(CHUNK, exception_on_overflow=False)
            np_data = np.frombuffer(data, dtype=np.int16)
            frame_energy = np.mean(np.abs(np_data))
            # print(frame_energy)
            # 如果能量低于阈值持续时间过长，则停止录音
            if frame_energy < THRESHOLD:
                count += 1
            elif count > 0:
                count -= 1

            frames.append(data)
        # print("停止录音!")
        self.messageSignal.emit(f"{self.name()} 停止录音!")
        stream.stop_stream()
        stream.close()
        audio.terminate()

        rec = KaldiRecognizer(model, RATE)
        rec.SetWords(True)
        str_ret = ""
        for data in frames:
            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                if 'text' in result:
                    str_ret += result['text']

        result = json.loads(rec.FinalResult())
        if 'text' in result:
            str_ret += result['text']

        str_ret = "".join(str_ret.split())
        return str_ret

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

class WhisperRecognitionNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'Speech recognition by whisper'

    def __init__(self):
        super(WhisperRecognitionNode, self).__init__()
        # 初始化语音识别
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.add_output('text_out')
        self.text_out = ""

    def execute(self):
        with self.microphone as source:
            self.messageSignal.emit(f"{self.name()} 请说话...")
            self.recognizer.adjust_for_ambient_noise(source)  # 降噪
            audio = self.recognizer.listen(source)
        try:
            text = self.recognizer.recognize_whisper(audio, language='zh')  # 中文识别
            self.messageSignal.emit(f"{self.name()} Output result: {text}")
            self.text_out = text
        except sr.UnknownValueError:
            self.messageSignal.emit(f"{self.name()} Output result: 无法识别语音")
            return ""
        except sr.RequestError:
            self.messageSignal.emit(f"{self.name()} Output result: 语音服务不可用")
            return ""
        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

class Pyttsx3SpeakNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'Speak by pyttsx3 '

    def __init__(self):
        super(Pyttsx3SpeakNode, self).__init__()
        # 初始化语音引擎
        self.engine = pyttsx3.init()
        # 设置语音属性（可选）
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', voices[0].id)  # 0通常是英文男声，1可能是英文女声，中文可能需要其他设置
        self.engine.setProperty('rate', 150)  # 语速
        self.add_input('text_in')

        self.add_checkbox('isConversationLoop', text='是否开启对话循环')

    def execute(self):
        """"""
        text = self.input(0).connected_ports()[0].node().text_out
        self.messageSignal.emit(f"{self.name()} Output result: {text}")
        self.engine.say(text)
        self.engine.runAndWait()

        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

        if self.get_property('isConversationLoop'):
            # 遍历前置节点，并保存到一个列表中
            self.get_execution_order(self) # 遍历前置节点，再执行它们


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

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal