# See here for ROS 2 versions and recommended Ubuntu versions:
# https://docs.ros.org/en/foxy/Releases.html

# Settings
ARG WEBTOP_IMAGE=ubuntu-xfce-version-d5ad760c
ARG ROS_DISTRO=jazzy
ARG DEFAULT_USERNAME=abc
ARG DEFAULT_PASSWORD=abc
ARG WGET_ARGS="-q --show-progress --progress=bar:force:noscroll"
ARG VS_CODE_EXT_ROS_VERSION=0.9.6
ARG VS_CODE_EXT_PYTHON_VERSION=2025.5.2025040401
ARG VS_CODE_EXT_CPPTOOLS_VERSION=1.24.5
ARG VS_CODE_EXT_HEX_EDITOR_VERSION=1.11.1
ARG VS_CODE_EXT_CMAKETOOLS_VERSION=1.21.9
ARG VS_CODE_EXT_CPP_EXT_PACK_VERSION=1.3.1

#-------------------------------------------------------------------------------
# Base image and dependencies

# Base image
FROM linuxserver/webtop:${WEBTOP_IMAGE}

# Redeclare arguments after FROM
ARG ROS_DISTRO
ARG DEFAULT_USERNAME
ARG DEFAULT_PASSWORD
ARG TARGETARCH
ARG WGET_ARGS
ARG VS_CODE_EXT_ROS_VERSION
ARG VS_CODE_EXT_PYTHON_VERSION
ARG VS_CODE_EXT_CPPTOOLS_VERSION
ARG VS_CODE_EXT_HEX_EDITOR_VERSION
ARG VS_CODE_EXT_CMAKETOOLS_VERSION
ARG VS_CODE_EXT_CPP_EXT_PACK_VERSION

# Set default shell during Docker image build to bash
SHELL ["/bin/bash", "-c"]

# Check if the target architecture is either x86_64 (amd64) or arm64 (aarch64)
RUN if [ "$TARGETARCH" = "amd64" ] || [ "$TARGETARCH" = "arm64" ]; then \
        echo "Architecture $TARGETARCH is supported."; \
    else \
        echo "Unsupported architecture: $TARGETARCH"; \
        exit 1; \
    fi

# Install dependencies
RUN apt-get -y update && \
    apt-get -y install \
        curl \
        locales \
        nano \
        openssh-server \
        python3 \
        python3-pip \
        python3-venv \
        software-properties-common

# Set the locale
RUN locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Set up workspace directory
RUN mkdir -p /config/workspaces && \
    chown -R ${DEFAULT_USERNAME}:${DEFAULT_USERNAME} /config/workspaces && \
    chmod -R 0777 /config/workspaces

# Set up sshd working directory
RUN mkdir -p /var/run/sshd && \
    chmod 0755 /var/run/sshd

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

# Change permissions for ROS 2 source directory
RUN chmod -R 0777 /opt/ros/${ROS_DISTRO}

# Set environment variables for ROS 2
ENV ROS_DISTRO=${ROS_DISTRO}

#-------------------------------------------------------------------------------
# VS Code

# Add Microsoft GPG key and repository
RUN cd /tmp && \
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg && \
    install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg && \
    echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | tee /etc/apt/sources.list.d/vscode.list > /dev/null && \
    rm -f packages.microsoft.gpg

# Update package list and install VS Code
RUN apt-get update && \
    apt-get install -y apt-transport-https && \
    apt-get install -y code && \
    rm -rf /var/lib/apt/lists/*

# Suppress WSL warning in VS Code when launched from CLI
ENV DONT_PROMPT_WSL_INSTALL=true

# Download VS Code extensions
RUN mkdir -p /tmp/vscode-extensions && \
    cd /tmp/vscode-extensions && \
    wget --compression=gzip ${WGET_ARGS} https://marketplace.visualstudio.com/_apis/public/gallery/publishers/ms-iot/vsextensions/vscode-ros/${VS_CODE_EXT_ROS_VERSION}/vspackage -O ros.vsix && \
    wget --compression=gzip ${WGET_ARGS} https://marketplace.visualstudio.com/_apis/public/gallery/publishers/ms-python/vsextensions/python/${VS_CODE_EXT_PYTHON_VERSION}/vspackage -O python.vsix && \
    wget --compression=gzip ${WGET_ARGS} https://marketplace.visualstudio.com/_apis/public/gallery/publishers/ms-vscode/vsextensions/hexeditor/${VS_CODE_EXT_HEX_EDITOR_VERSION}/vspackage -O hexeditor.vsix && \
    wget --compression=gzip ${WGET_ARGS} https://marketplace.visualstudio.com/_apis/public/gallery/publishers/ms-vscode/vsextensions/cpptools-extension-pack/${VS_CODE_EXT_CPP_EXT_PACK_VERSION}/vspackage -O cppextpack.vsix
    

# Install VS Code extensions
RUN cd /tmp/vscode-extensions && \
    code --install-extension ros.vsix --force --user-data-dir /root/.vscode-root --no-sandbox && \
    code --install-extension python.vsix --force --user-data-dir /root/.vscode-root --no-sandbox && \
    code --install-extension hexeditor.vsix --force --user-data-dir /root/.vscode-root --no-sandbox && \
    code --install-extension cppextpack.vsix --force --user-data-dir /root/.vscode-root --no-sandbox

# Clean up VS Code extensions
RUN rm -rf /tmp/vscode-extensions

# Copy workspace configuration
COPY .scripts/ros2.code-workspace /ros2.code-workspace

# Replace ROS distro in workspace configuration
RUN sed -i 's/@@ROS_DISTRO@@/'"$ROS_DISTRO"'/g' /ros2.code-workspace

# Change permissions for .vscode directory and workspace configuration
RUN chmod -R 0777 /config/.vscode && \
    chmod 0777 /ros2.code-workspace

# Change permissions for cache directory
RUN mkdir -p /config/.cache && \
    chown -R ${DEFAULT_USERNAME}:${DEFAULT_USERNAME} /config/.cache && \
    chmod -R 0777 /config/.cache

#-------------------------------------------------------------------------------
# Desktop customization

# Copy VS Code Desktop launcher
RUN mkdir -p /config/.icons
COPY .images/vscode-ros-icon_512px.png /config/.icons/vscode-ros-icon.png
COPY .scripts/vscode-ros2.desktop /config/Desktop/vscode-ros2.desktop
RUN chown ${DEFAULT_USERNAME}:${DEFAULT_USERNAME} /config/Desktop/vscode-ros2.desktop && \
    chmod 0777 /config/Desktop/vscode-ros2.desktop

#-------------------------------------------------------------------------------
# Entrypoint

# Custom entrypoint
COPY .scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
