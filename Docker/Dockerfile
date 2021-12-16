FROM osrf/ros:noetic-desktop-full

RUN apt-get update -q \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    curl \
    git \
    libgl1-mesa-dev \
    libgl1-mesa-glx \
    libglew-dev \
    libosmesa6-dev \
    libglfw3 \
    software-properties-common \
    net-tools \
    unzip \
    vim \
    wget \
    xpra \
    xserver-xorg-dev \
    x11proto-gl-dev \
    swig \
    python3-pkgconfig \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN curl -o /usr/local/bin/patchelf https://s3-us-west-2.amazonaws.com/openai-sci-artifacts/manual-builds/patchelf_0.9_amd64.elf \
    && chmod +x /usr/local/bin/patchelf

ENV LANG C.UTF-8

RUN mkdir -p /root/.mujoco \
    && wget https://www.roboti.us/download/mujoco200_linux.zip -O mujoco.zip \
    && unzip mujoco.zip -d /root/.mujoco \
    && mv /root/.mujoco/mujoco200_linux /root/.mujoco/mujoco200 \
    && rm mujoco.zip
COPY ./mjkey.txt /root/.mujoco/
ENV LD_LIBRARY_PATH /root/.mujoco/mujoco200/bin:${LD_LIBRARY_PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
ENV MUJOCO_GL=egl
ENV PYOPENGL_PLATFORM=egl

COPY ./requirements.txt /
RUN pip3 install --upgrade pip wheel setuptools
RUN pip3 install -r /requirements.txt

RUN mkdir -p /ros_ws/src
RUN /bin/bash -c "cd /ros_ws/src && git clone https://github.com/Roboy/roboy_communication.git" 
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /ros_ws && catkin_make" 

RUN echo "source /opt/ros/noetic/setup.bash" > /root/.bashrc
RUN echo "source /ros_ws/devel/setup.bash" > /root/.bashrc

ENV LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so

