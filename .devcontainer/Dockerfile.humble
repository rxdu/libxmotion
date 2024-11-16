FROM osrf/ros:humble-desktop

# Set up user account
RUN useradd -ms /bin/bash ros && echo "ros:ros" | chpasswd && adduser ros sudo

# Install dependent packages
ENV DEBIAN_FRONTEND=noninteractive
RUN sudo apt-get update && \
        sudo apt-get -y install --no-install-recommends \
        ros-humble-ackermann-msgs \
        ros-humble-serial-driver
RUN sudo apt-get install -y \
        libgl1-mesa-dev libglfw3-dev libcairo2-dev libtbb-dev libasio-dev \
        libboost-all-dev libgsl-dev libeigen3-dev libtbb-dev libopencv-dev libyaml-cpp-dev

# Clean up
RUN sudo apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]