ARG DOCKER_TAG=24.04
FROM docker.1ms.run/ubuntu:${DOCKER_TAG}

# 执行命令时，不需要交互，直接运行
ARG DEBIAN_FRONTEND=nointeractive

RUN apt-get update \
	  && apt install -y -q \
	  cmake \
	  git \
	  python3 \
	  python3-pip \
	  wget \
	  x11-apps \
	  sudo \
	  python3-venv \
	  vim

CMD ["/bin/bash"]