# See here for ROS 2 versions and recommended Ubuntu versions:
# https://docs.ros.org/en/foxy/Releases.html

# Settings
ARG UBUNTU_VERSION=24.04
ARG ROS_DISTRO=jazzy

#-------------------------------------------------------------------------------
# Base image and dependencies

# Base image
FROM ubuntu:${UBUNTU_VERSION}

# Redeclare arguments after FROM
ARG ROS_DISTRO

# Install dependencies
RUN apt-get -y update && \
    apt-get -y install \
        curl \
        locales \
        openssh-server \
        software-properties-common \
        curl

# Set the locale
RUN locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Set up directories
RUN mkdir -p /workspaces/.vscode && \
    mkdir -p /opt/toolchains

# Set up sshd working directory
RUN mkdir -p /var/run/sshd && \
    chmod 0755 /var/run/sshd

# Allow root login via SSH
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config

# Expose SSH port
EXPOSE 22

#-------------------------------------------------------------------------------
# ROS 2

# Enable Ubuntu Universe repository
RUN add-apt-repository universe

# Add ROS 2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to source list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
RUN apt-get -y update && \
    apt-get -y install \
        ros-dev-tools \
        ros-${ROS_DISTRO}-desktop

# Source ROS 2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Clean up stale packages
RUN apt-get clean -y && \
	apt-get autoremove --purge -y && \
	rm -rf /var/lib/apt/lists/*

#-------------------------------------------------------------------------------
# Entrypoint

# Custom entrypoint
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
