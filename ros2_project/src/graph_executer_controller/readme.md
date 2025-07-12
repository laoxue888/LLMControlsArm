
---

[TOC]

# 前言

配置运行graph_executer_controller的开发环境


# 操作

❇️配置开发环境

```shell
sudo apt update
apt install python3-pip -y
apt-get install portaudio19-dev -y
apt-get install x11-xserver-utils
apt install libxcb* -y

# 使用清华源下载
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple --break-system-packages
```

❇️运行

```shell
cd ros2_docker_ws
source install/setup.bash

cd src/graph_executer_controller
python3 main.py

```
