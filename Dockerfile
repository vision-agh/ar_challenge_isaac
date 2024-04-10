FROM nvcr.io/nvidia/isaac-sim:2023.1.1
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip git curl
ENV SHELL /bin/bash
RUN curl -fsSL https://get.docker.com -o get-docker.sh
RUN chmod +x get-docker.sh
RUN ./get-docker.sh

RUN locale 

RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get install -y software-properties-common
RUN add-apt-repository universe

RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update
RUN apt upgrade
RUN apt install ros-humble-desktop python3-argcomplete -y
RUN apt install ros-humble-vision-msgs -y
RUN apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
RUN apt install python3-colcon-common-extensions -y
RUN apt install git make cmake python3-pip -y
RUN pip install kconfiglib jinja2 empy jsonschema pyros-genmsg packaging toml numpy future
RUN apt install python3-rocker -y

RUN apt install nano -y
RUN apt install ros-humble-ament-cmake-clang-format -y
RUN apt install ros-humble-rmw-fastrtps-* -y