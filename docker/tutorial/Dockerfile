FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04

LABEL maintainer "Takaki Ueno <ueknot@gmail.com>"

ARG username
ARG userid
ARG groupid

#Add new sudo user
RUN useradd -m $username && \
        echo "$username:$username" | chpasswd && \
        usermod --shell /bin/bash $username && \
        usermod -aG sudo $username && \
        mkdir -p /etc/sudoers.d && \
        echo "$username ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$username && \
        chmod 0440 /etc/sudoers.d/$username && \
        usermod  --uid $userid $username && \
        groupmod --gid $groupid $username

# add user to dialout group
RUN usermod -a -G dialout $username

# upgrade packages
RUN apt update && apt upgrade -y
RUN apt install -y lsb-release

# setup ROS repository key
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install ROS kinetic
RUN apt update
RUN apt install -y ros-kinetic-desktop-full \
                   python-rosinstall
RUN apt install -y python-wstool \
                   python-rosinstall-generator \
                   python-catkin-tools

# install common deps
RUN curl -sSL https://raw.githubusercontent.com/PX4/Devguide/v1.8.0/build_scripts/ubuntu_sim_common_deps.sh | bash

# update rosdep
RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep init
RUN rosdep update

# switch user not to run rosdep update with root privilege
USER ${username}
RUN rosdep update

RUN mkdir -p /home/${username}/catkin_ws/src
WORKDIR /home/${username}/catkin_ws

RUN wstool init /home/${username}/catkin_ws/src

RUN rosinstall_generator --rosdistro kinetic --upstream mavros | tee /tmp/mavros.rosinstall
RUN rosinstall_generator --rosdistro kinetic mavlink | tee -a /tmp/mavros.rosinstall
RUN wstool merge -t src /tmp/mavros.rosinstall
RUN wstool update -t src

# install dependencies
USER root
RUN rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --os ubuntu:xenial

# build catkin workspace
USER ${username}
RUN bash -c "source /opt/ros/kinetic/setup.bash && catkin build"
RUN echo "source /home/${username}/catkin_ws/devel/setup.bash" >> /home/${username}/.bashrc

# install geographiclib
USER root
RUN curl -sSL https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash

# update gazebo so that gazebo is newer than 7.4
RUN apt purge gazebo* libgazebo* -y
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

RUN apt install wget
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

RUN apt update
RUN apt install gazebo7 libgazebo7-dev -y

# install opencv dependencies
WORKDIR /home/${username}
RUN apt install build-essential -y
RUN apt install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev -y
RUN apt install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev -y

# clone opencv 3.4.12
USER ${username}
RUN git clone https://github.com/opencv/opencv.git -b 3.4.12
RUN git clone https://github.com/opencv/opencv_contrib.git -b 3.4.12

# build and install opnecv
WORKDIR /home/${username}/opencv
RUN mkdir build

WORKDIR /home/${username}/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..

USER root
RUN make clean && make && make install

# install dronedoc dependencies
RUN apt install protobuf-compiler libignition-math2-dev -y
RUN apt install ros-kinetic-amcl \
                ros-kinetic-base-local-planner \
                ros-kinetic-carrot-planner \
                ros-kinetic-clear-costmap-recovery \
                ros-kinetic-costmap-2d \
                ros-kinetic-dwa-local-planner \
                ros-kinetic-fake-localization \
                ros-kinetic-global-planner \
                ros-kinetic-map-server \
                ros-kinetic-move-base \
                ros-kinetic-move-base-msgs \
                ros-kinetic-move-slow-and-clear \
                ros-kinetic-nav-core \
                ros-kinetic-navfn \
                ros-kinetic-rotate-recovery \
                ros-kinetic-voxel-grid -y
RUN apt install ros-kinetic-hector-slam -y
RUN apt install ros-kinetic-octomap-mapping -y
# RUN apt install ros-kinetic-turtlebot-teleop \
#                 ros-kinetic-turtlebot-gazebo \
#                 ros-kinetic-turtlebot-bringup -y
RUN apt install ros-kinetic-moveit-* -y
RUN apt install ros-kinetic-ompl -y
RUN apt install ros-kinetic-eband-local-planner \
                ros-kinetic-teb-local-planner -y
RUN apt install ros-kinetic-gazebo-ros-pkgs -y

# clone dronedoc and build
USER ${username}
WORKDIR /home/${username}/catkin_ws/src
RUN git clone https://github.com/uenota/dronedoc.git
RUN catkin build

# clone PX4/Firmware
WORKDIR /home/${username}
RUN rm -rf src/Firmware
RUN mkdir -p src

WORKDIR /home/${username}/src
RUN git clone https://github.com/PX4/Firmware.git -b v1.8.0

# build PX4 firmware
WORKDIR /home/${username}/src/Firmware
RUN make rostest

# make bash to run setup script for PX4 Firmware every time .bashrc is loaded
RUN echo 'source $HOME/src/Firmware/Tools/setup_gazebo.bash $HOME/src/Firmware $HOME/src/Firmware/build/posix_sitl_default > /dev/null' >> /home/${username}/.bashrc

# add PX4 Firmware path to ROS package path
RUN echo 'ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/src/Firmware' >> /home/${username}/.bashrc
RUN echo 'ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/src/Firmware/Tools/sitl_gazebo' >> /home/${username}/.bashrc

# add Gazebo models from PX4 Firmware and dronedoc to Gazebo model path
RUN echo 'export GAZEBO_MODEL_PATH=$HOME/src/Firmware/Tools/sitl_gazebo/models:$GAZEBO_MODEL_PATH' >> /home/${username}/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/dronedoc/models:$GAZEBO_MODEL_PATH' >> /home/${username}/.bashrc

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# install packages for test and development
USER root
RUN apt install vim tmux -y

CMD ["bash"]
