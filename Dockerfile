# Set desired ROS distribution
ARG ROS_DISTRO=jazzy

# The base image for the overlay deployment
ARG ROS_WS_BASE_IMAGE_TAG="latest"
ARG ROS_WS_BASE_IMAGE="togo_docker_ws-dev"
ARG ROS_WS_BASE_IMAGE="${ROS_WS_BASE_IMAGE}:${ROS_WS_BASE_IMAGE_TAG}"


# This layer grabs package manifests from the src directory for preserving rosdep installs.
# This can significantly speed up rebuilds for the base package when src contents have changed.

FROM alpine:latest AS package-manifests

# Copy in the src directory, then remove everything that isn't a manifest or an ignore file.
COPY src/ /src/
RUN find /src -type f ! -name "package.xml" ! -name "COLCON_IGNORE" -delete && \
    find /src -type d -empty -delete

# Throw away for an empty source directory
RUN mkdir -p /src


# Grab SLAM package manifests for rosdep caching

FROM alpine:latest AS slam-manifests

COPY lidarslam_ros2/ /slam_src/
RUN find /slam_src -type f ! -name "package.xml" ! -name "COLCON_IGNORE" -delete && \
    find /slam_src -type d -empty -delete


# Using the pre-compiled ROS images as the base.

FROM osrf/ros:${ROS_DISTRO}-desktop-full AS er4-dev

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Overridable non root user information.
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=er4-user

# Define the install location for the developing application
ENV ER4_WS="/home/er4-user/ws"
ENV SLAM_WS="/home/er4-user/slam_ws"

# DEBIAN_FRONTEND is set as an ARG instead of ENV variable so it doesn't persist in the image after build
ARG DEBIAN_FRONTEND=noninteractive

# As of 24.04, many Ubuntu modules will check for FIPS kernels and adjust packages accordingly. This
# can break in the container, which shares a kernel but does not have FIPS packages installed. So
# in the running image we ensure that SSL does not cause problems when downloading or making
# secure connections during the build.
ENV OPENSSL_FORCE_FIPS_MODE=0


# Base dev tools

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -q -y \
    bash-completion \
    ccache \
    gdb \
    gdbserver \
    git \
    less \
    python3-colcon-clean \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    software-properties-common \
    terminator \
    tmux \
    vim \
    xterm \
    wget \
    nvidia-cuda-dev \
    nvidia-cuda-toolkit \
    mesa-utils \
    ros-${ROS_DISTRO}-sophus \
    gedit


# Add a non-root user with provided user details. Some images have a default `ubuntu` user,
# so we remove it before adding the new one.

RUN userdel -r ubuntu 2>/dev/null || true
RUN groupadd -g ${USER_GID} ${USERNAME} \
    && useradd -l -u ${USER_UID} -g ${USER_GID} --create-home -m -s /bin/bash \
       -G sudo,adm,dialout,dip,plugdev,video ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    mkdir -p \
        /home/${USERNAME}/.ccache \
        /home/${USERNAME}/.colcon \
        /home/${USERNAME}/.ros \
        /home/${USERNAME}/.bash \
        ${ER4_WS}/src \
        ${ER4_WS}/build \
        ${ER4_WS}/install \
        ${ER4_WS}/log \
        ${SLAM_WS}/src \
        ${SLAM_WS}/build \
        ${SLAM_WS}/install \
        ${SLAM_WS}/log && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

# Add the Clearpath Robotics public package signing key
RUN wget https://packages.clearpathrobotics.com/public.key -O - | apt-key add -

# Add the Clearpath stable package repository
RUN sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/clearpath-latest.list'

# Add Clearpath custom rosdep rules
RUN wget https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list \
    -O /etc/ros/rosdep/sources.list.d/50-clearpath.list


# Sim workspace rosdeps

USER ${USERNAME}
WORKDIR ${ER4_WS}

# Copy package manifests for installing rosdeps
COPY --chown=${USERNAME}:${USERNAME} --from=package-manifests /src/ ./src

# Install rosdeps
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt update && \
    . /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install -iyr --from-paths src

# Install extra ROS deps
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update && \
    sudo apt-get install -q -y \
    ros-${ROS_DISTRO}-ros2controlcli \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-teleop-twist-keyboard

# Copy in the remainder of the src directory
COPY --chown=${USERNAME}:${USERNAME} src/ src/


# SLAM workspace rosdeps + GTSAM build deps

WORKDIR ${SLAM_WS}

# Copy SLAM package manifests for installing rosdeps
COPY --chown=${USERNAME}:${USERNAME} --from=slam-manifests /slam_src/ ./src/lidarslam_ros2/

# Install SLAM rosdeps and GTSAM build dependencies.
# Note: libgtsam-dev is NOT used here — we build GTSAM from vendored source
# in Thirdparty/gtsam/ because the Ubuntu Noble apt package is incompatible
# with system Eigen 4.0 (shipped with ROS 2 Jazzy).
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt update && \
    . /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install -iyr --from-paths src && \
    sudo apt-get install -q -y \
        libg2o-dev \
        libboost-all-dev \
        cmake

# Copy full SLAM source, which includes vendored GTSAM 4.3a2 in Thirdparty/gtsam/
COPY --chown=${USERNAME}:${USERNAME} lidarslam_ros2/ ./src/lidarslam_ros2/

# Build and install vendored GTSAM 4.3a2 from source.
# GTSAM_USE_SYSTEM_EIGEN=ON ensures Eigen version consistency with ROS 2 Jazzy packages.
RUN cd ${SLAM_WS}/src/lidarslam_ros2/Thirdparty/gtsam && \
    mkdir build && cd build && \
    cmake .. \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    sudo make install && \
    sudo ldconfig && \
    touch ${SLAM_WS}/src/lidarslam_ros2/Thirdparty/gtsam/COLCON_IGNORE

# Build the SLAM workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release


# Setup colcon default mixins and add default settings

WORKDIR ${ER4_WS}

RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update || true
RUN colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update || true

# Copy in configs for different features
COPY --chown=${USERNAME}:${USERNAME} config/colcon-defaults.yaml /home/${USERNAME}/.colcon/defaults.yaml
COPY --chown=${USERNAME}:${USERNAME} config/terminator_config /home/${USERNAME}/.config/terminator/config

# Setup entrypoint and ensure it's added to ~/.bashrc
COPY scripts/entrypoint.sh /entrypoint.sh
RUN echo "source /entrypoint.sh" >> ~/.bashrc

# Make it obvious when operating in a container
RUN echo "PS1=\"${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\](docker):\[\033[01;34m\]\w\[\033[00m\]\$ \"" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]


# Source built dev image for automated testing.

FROM er4-dev AS er4-dev-source

ARG USERNAME

RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${ER4_WS} && colcon build && \
    . ${ER4_WS}/install/setup.bash && \
    cd ${SLAM_WS} && colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release


# er4-robot: production deploy target

FROM ${ROS_WS_BASE_IMAGE} AS er4-robot

ARG USERNAME
ARG USER_UID
ARG USER_GID

USER root

RUN OLD_UID=$(id -u ${USERNAME}) && \
    OLD_GID=$(id -g ${USERNAME}) && \
    if [ "${OLD_UID}" != "${USER_UID}" ] || [ "${OLD_GID}" != "${USER_GID}" ]; then \
        sed -i "s/^\(${USERNAME}:[^:]*:\)[^:]*:[^:]*:/\1${USER_UID}:${USER_GID}:/" /etc/passwd && \
        sed -i "s/^\(${USERNAME}:[^:]*:\)[^:]*:/\1${USER_GID}:/" /etc/group && \
        find /home/${USERNAME} \
            \( -user ${OLD_UID} -o -group ${OLD_GID} \) \
            -print0 | xargs -0 -P $(nproc) -n 1000 chown ${USER_UID}:${USER_GID}; \
    fi

USER ${USERNAME}
