
---

[TOC]

# 前言

通过使用智能化的工作流控制系统来精确操控机械臂，不仅能够基于预设算法可靠地规划每个动作步骤的执行顺序和力度，确保作业流程的标准化和可重复性，还能通过模块化的程序设计思路灵活地在原有工作流中插入新的控制节点，这种可扩展的架构设计使得系统能够在不影响既有功能稳定性的前提下，便捷地集成诸如视觉识别、力反馈调节或协同作业等进阶功能模块，从而持续提升机械臂在复杂工业场景中的适应性和多功能性。

这里我增加了DeepSeek控制节点，通过给DeepSeek提示，让它帮助给出机械臂的末端位置。该节点利用大语言模型强大的逻辑推理和空间理解能力，能够将模糊的自然语言指令（如“向右移动5厘米”或“避开红色障碍物”）自动转化为精确的坐标参数和运动轨迹。这一集成显著提升了人机交互效率，使得非专业用户也能通过口语化指令快速完成复杂的位姿调整任务，为柔性生产线或科研实验场景提供了更高层次的智能化支持。

> 参考：
> - [JSON Output](https://api-docs.deepseek.com/zh-cn/guides/json_mode)
> - [DeepSeek提示库](https://api-docs.deepseek.com/zh-cn/prompt-library/)

# 环境配置

- Ubuntu:24.04
- Ros2:jazzy


❇️使用VMWare安装Ubuntu24.04


❇️配置开发环境

```shell
# VMware的复制粘贴问题：先打开VMware的设置，点开选项->常规->共享文件夹->勾选启用共享文件夹

sudo apt-get autoremove open-vm-tools -y
sudo apt-get install open-vm-tools -y
sudo apt-get install open-vm-tools-desktop -y
sudo reboot


# 按照鱼香ros一键安装ros2
sudo apt-get update
sudo apt install wget -y
wget http://fishros.com/install -O fishros && sudo bash fishros

# 安装远程显示服务程序
sudo apt-get install x11-xserver-utils
sudo apt install libxcb* -y
sudo apt-get install x11-apps -y

# 安装moveit
# 重新打开一个终端
sudo apt install ros-${ROS_DISTRO}-moveit* -y

# 安装ros2的控制功能包
sudo apt install ros-${ROS_DISTRO}-controller-manager -y
sudo apt install ros-${ROS_DISTRO}-joint-trajectory-controller -y
sudo apt install ros-${ROS_DISTRO}-joint-state-broadcaster -y
sudo apt install ros-${ROS_DISTRO}-diff-drive-controller -y

# 安装运行`graph_executer_controller`相关的Python包
sudo apt update
sudo apt install python3-pip -y
sudo apt-get install portaudio19-dev -y
sudo apt install espeak -y

# 使用清华源下载
cd src/graph_executer_controller/
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple --break-system-packages

# 调试工具
python3 -m pip install ipykernel -U --user --force-reinstall -i https://pypi.tuna.tsinghua.edu.cn/simple --break-system-packages


# 配置Ubuntu支持中文
sudo apt-get install language-pack-zh-hans

vim /etc/environment
# 添加内容
LANG="zh_CN.UTF-8"
LANGUAGE="zh_CN:zh:en_US:en"

vim /var/lib/locales/supported.d/local
# 添加内容
en_US.UTF-8 UTF-8
zh_CN.UTF-8 UTF-8
zh_CN.GBK GBK
zh_CN GB2312

sudo locale-gen

# 安装中文字体
sudo apt-get install fonts-droid-fallback ttf-wqy-zenhei ttf-wqy-microhei fonts-arphic-ukai fonts-arphic-uming
```

# 运行测试

❇️编译项目

```shell    
colcon build
```

❇️运行`moveitpy_controller`

```shell
# 打开新的终端
source install/setup.bash
ros2 launch control_server arm_control.launch.py
```


❇️运行`panda_moveit_config`的`demo.launch.py`

```shell
# 打开新的终端
source install/setup.bash
ros2 launch panda_moveit_config demo.launch.py
```
启动rviz2后，可以看到机械臂会有干涉，现手动调整到不干涉的位置，然后才使用moveitpy控制机械臂，否则无法控制机械臂。


❇️运行`graph_executer_controller`

```shell
# 打开新的终端
source install/setup.bash
cd src/graph_executer_controller
python3 main.py
```

# 报错

1. ❌框选节点的时候报错

```shell
  File "/usr/local/lib/python3.12/dist-packages/NodeGraphQt/widgets/viewer.py", line 619, in mouseMoveEvent
    self.scene().setSelectionArea(
TypeError: 'PySide6.QtWidgets.QGraphicsScene.setSelectionArea' called with wrong argument types:
  PySide6.QtWidgets.QGraphicsScene.setSelectionArea(QPainterPath, ItemSelectionMode)
Supported signatures:
  PySide6.QtWidgets.QGraphicsScene.setSelectionArea(path: PySide6.QtGui.QPainterPath, /, selectionOperation: PySide6.QtCore.Qt.ItemSelectionOperation = Instance(Qt.ReplaceSelection), mode: PySide6.QtCore.Qt.ItemSelectionMode = Instance(Qt.IntersectsItemShape), deviceTransform: PySide6.QtGui.QTransform = Default(QTransform))
  PySide6.QtWidgets.QGraphicsScene.setSelectionArea(path: PySide6.QtGui.QPainterPath, deviceTransform: PySide6.QtGui.QTransform, /)
Error calling Python override of QGraphicsView::mouseMoveEvent(): Traceback (most recent call last):
  File "/usr/local/lib/python3.12/dist-packages/NodeGraphQt/widgets/viewer.py", line 619, in mouseMoveEvent
    self.scene().setSelectionArea(
TypeError: 'PySide6.QtWidgets.QGraphicsScene.setSelectionArea' called with wrong argument types:
  PySide6.QtWidgets.QGraphicsScene.setSelectionArea(QPainterPath, ItemSelectionMode)
Supported signatures:
  PySide6.QtWidgets.QGraphicsScene.setSelectionArea(path: PySide6.QtGui.QPainterPath, /, selectionOperation: PySide6.QtCore.Qt.ItemSelectionOperation = Instance(Qt.ReplaceSelection), mode: PySide6.QtCore.Qt.ItemSelectionMode = Instance(Qt.IntersectsItemShape), deviceTransform: PySide6.QtGui.QTransform = Default(QTransform))
  PySide6.QtWidgets.QGraphicsScene.setSelectionArea(path: PySide6.QtGui.QPainterPath, deviceTransform: PySide6.QtGui.QTransform, /)
```

✔️Pyside6和nodegraphqt版本不适配导致，更改`viewer.py`以下位置即可：

```shell
# "/usr/local/lib/python3.12/dist-packages/NodeGraphQt/widgets/viewer.py"

# self.scene().setSelectionArea(
#     path, QtCore.Qt.IntersectsItemShape
# )
self.scene().setSelectionArea(
    path,
    selectionOperation=QtCore.Qt.ItemSelectionOperation.ReplaceSelection,
    mode=QtCore.Qt.ItemSelectionMode.IntersectsItemShape
)
```