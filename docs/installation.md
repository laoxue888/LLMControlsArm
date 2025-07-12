


❇️创建Docker容器

```shell
docker run -it -p 6080:80 -p 10000:10000 -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=host.docker.internal:0.0 -e PULSE_SERVER=host.docker.internal --name=LLMControlsArm docker.1ms.run/ubuntu:24.04  /bin/bash
```

❇️配置开发环境

```shell
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

❇️安装NodeGraphQt

```shell
git clone https://github.com/laoxue888/NodeGraphQt.git
cd NodeGraphQt
pip install -e .
```