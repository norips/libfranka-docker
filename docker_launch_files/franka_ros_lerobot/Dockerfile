FROM ros:noetic

### Make installation non interactive so that docker doesnt crash during build
ENV DEBIAN_FRONTEND noninteractive

### Basic OS setup
RUN apt-get update -y &&\
    apt-get install -y \
        git \
        cmake \
        python3-pip \
        build-essential \
        libeigen3-dev \
        libpoco-dev

### ROS setup
RUN apt-get update -y &&\
    apt-get install -y \
        ros-noetic-rviz

### Install display mounting tools
RUN apt-get update -y &&\
    apt-get install -y  \
        python3-tk \
        python3-gi-cairo


### Set Working Directory    
WORKDIR /docker_volume


### Install Libfranka
COPY ./install_franka_ros.sh /install_franka_ros.sh
RUN chmod +x /install_franka_ros.sh

### Install rqt-graph for debugging & wget for conda
RUN apt-get update && apt-get install -y \
    ros-noetic-rqt-graph \
    wget

### Install conda
RUN arch=$(uname -m) && \
    if [ "$arch" = "x86_64" ]; then \
    MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh"; \
    elif [ "$arch" = "aarch64" ]; then \
    MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh"; \
    else \
    echo "Unsupported architecture: $arch"; \
    exit 1; \
    fi && \
    wget $MINICONDA_URL -O miniconda.sh && \
    mkdir -p /root/.conda && \
    bash miniconda.sh -b -p /root/miniconda3 && \
    rm -f miniconda.sh

ENV PATH="${PATH}:/root/miniconda3/bin"

### Install lerobot
RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
RUN conda create -y -n lerobot python=3.10
RUN conda run --no-capture-output -n lerobot pip install lerobot && \
    conda run --no-capture-output -n lerobot pip install 'lerobot[feetech, kinematics]'

RUN conda run --no-capture-output -n lerobot pip install zmq scipy

RUN python3.8 -m pip install zmq

RUN apt-get update && apt-get install -y ros-noetic-moveit-visual-tools ros-noetic-moveit-ros-move-group ros-noetic-moveit

COPY docker_volume/catkin_ws/src /tmp/catkin_ws/src
RUN cd /tmp/catkin_ws && rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka && cd / && rm -rf /tmp/catkin_ws/src

RUN ln /opt/ros/noetic/lib/moveit_ros_move_group/move_group /opt/ros/noetic/share/moveit_ros_move_group/move_group
RUN ln /opt/ros/noetic/lib/moveit_servo/servo_server /opt/ros/noetic/share/moveit_servo/servo_server

ENTRYPOINT [ "/install_franka_ros.sh" ]

CMD ["/bin/bash"]
