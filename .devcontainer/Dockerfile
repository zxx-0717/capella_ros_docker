FROM registry.cn-hangzhou.aliyuncs.com/ritju/ros2:humble-full AS full

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
COPY "files/ros2.list" "/etc/apt/sources.list.d/"
COPY "files/sources.list" "/etc/apt/"
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update \
    && apt-get -y install --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    python3-pytest \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* \
    && pip install rosdepc \
    && rosdepc init 
ENV DEBIAN_FRONTEND=dialog
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
RUN git config --global --add safe.directory $WORKSPACE
ENV ROBOT_VERSION=ad_2
ENV LASER_SCAN=laser_scan_wr_20
ENV LINEAR_SCALE=0.98859 
ENV ANGLE_SCALE=1.065448
ENV LASER_SCAN=laser_scan_wr_20
ENV LASER_MIN_ANGLE=-1.57
ENV LASER_MAX_ANGLE=1.57
ENV LASER_MIN_RANGE=0.2
ENV ROBOT_NAV_MODE=AD
ENV USE_TELEOP_TWIST_JOY=False
