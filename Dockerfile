FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies and MAVROS
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-setuptools \
    python3-colcon-common-extensions \
    git wget nano iproute2 \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    libgeographiclib-dev \
    geographiclib-tools \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Setup GeographicLib database for MAVROS
RUN mkdir -p /usr/share/GeographicLib \
    && geographiclib-get-geoids egm96-5 \
    && geographiclib-get-gravity egm96 \
    && geographiclib-get-magnetic wmm2020

# Install Python requirements
RUN pip3 install --no-cache-dir --break-system-packages future pymavlink dronekit mavproxy numpy pandas

# Setup custom user 'pilot' using the existing 'ubuntu' user
ARG USERNAME=pilot
RUN usermod -l $USERNAME ubuntu \
    && groupmod -n $USERNAME ubuntu \
    && usermod -d /home/$USERNAME -m $USERNAME \
    && usermod -aG sudo,dialout $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME/workspace

# Auto-source ROS2 and detect Windows default gateway IP
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "export WIN_IP=\$(ip route show default | awk '{print \$3}')" >> ~/.bashrc

