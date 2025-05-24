# Basic setup, install cyclonedds and rosdepc etc.
FROM althack/ros2:humble-full AS base
COPY ".devcontainer/files/ros2.list" "/etc/apt/sources.list.d/"
COPY ".devcontainer/files/sources.list" "/etc/apt/"
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' \
      --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt update \
    && apt-get -y install --no-install-recommends python3-pip \
      ros-humble-rmw-cyclonedds-cpp \
      wget \ 
    && pip install rosdepc \
    && rosdepc init \
    && rosdepc update

WORKDIR /build/src

# Packages defined in ritju/capella_msgs.
COPY ["src/capella_msgs/capella_ros_msg/package.xml", "capella_msgs/capella_ros_msg/package.xml"]
COPY ["src/capella_msgs/capella_ros_service_interfaces/package.xml", "capella_msgs/capella_ros_service_interfaces/package.xml"]

# Packages defined in ritju/navigation2.
COPY ["src/navigation2/nav2_amcl/package.xml", "navigation2/nav2_amcl/package.xml"]
COPY ["src/navigation2/nav2_dwb_controller/dwb_critics/package.xml", "navigation2/nav2_dwb_controller/dwb_critics/package.xml"]

# Packages defined in ritju/cartographer.
COPY ["src/cartographer/package.xml", "cartographer/package.xml"]

# Packages defined in ritju/cartographer-ros.
COPY ["src/cartographer-ros/cartographer_ros/package.xml", "cartographer-ros/cartographer_ros/package.xml"]

# Packages defined in in this repository.
COPY ["src/capella_cartographer_launcher/package.xml", "capella_cartographer_launcher/package.xml"]
COPY ["src/capella_charge_service/package.xml", "capella_charge_service/package.xml"]
COPY ["src/capella_nav_and_carto_server/package.xml", "capella_nav_and_carto_server/package.xml"]
COPY ["src/capella_ros_auto_charge_ctrl/package.xml", "capella_ros_auto_charge_ctrl/package.xml"]
COPY ["src/capella_ros_calib/package.xml", "capella_ros_calib/package.xml"]
COPY ["src/capella_ros_imu_calib/package.xml", "capella_ros_imu_calib/package.xml"]
COPY ["src/capella_ros_imu_filter/package.xml", "capella_ros_imu_filter/package.xml"]
COPY ["src/capella_ros_launcher/package.xml", "capella_ros_launcher/package.xml"]
COPY ["src/capella_ros_lidar_correction/package.xml", "capella_ros_lidar_correction/package.xml"]
COPY ["src/capella_ros_scan_odom/package.xml", "capella_ros_scan_odom/package.xml"]
COPY ["src/capella_ros_serial/package.xml", "capella_ros_serial/package.xml"]
COPY ["src/csm/package.xml", "csm/package.xml"]
COPY ["src/wj_716n_lidar/package.xml", "wj_716n_lidar/package.xml"]
COPY ["src/wr_ls_udp/package.xml", "wr_ls_udp/package.xml"]
COPY ["src/wr_ls_udp_20_H025/package.xml", "wr_ls_udp_20_H025/package.xml"]

# Install depedencies
RUN rosdepc install --from-paths . --ignore-src -y

# 以下为实际的编译步骤，分为4种：
# - build: 除了前面由rosdepc安装的包以外不依赖其他包，可以独立编译
# - build-base: 除了前面由rosdepc安装的包以外，还依赖capella_msgs里的包
# - build-python-packages: 与build-base相同，但是只包含python代码，不需要编译
# - build-python-packages-nodeps: 与build相同，但是只包含python代码，不需要编译
#
# 目前编译步骤的结构如下：
#
# - build
#   1 build-cartographer
#   2 build-navigation2
#   * build-capella_ros_scan_odom
#   * build-capella_ros_calib
#   * build-capella_ros_imu_filter
#   * build-capella_ros_lidar_correction
#   * build-wj_716n_lidar
#   * build-wr_ls_udp
#   * build-wr_ls_udp_high
#   * build-base
#     * build-capella_ros_serial
#     * build-capella_ros_auto_charge_ctrl
#     * build-capella_ros_imu_calib
#     * build-python-packages
#   * build-python-packages-nodeps
#
# 其中数字标记的步骤为顺序编译步骤，星号标注的步骤为并行编译步骤。
#
# 由于C++编译需要大量内存，为防止内存不足导致编译失败，所以大型C++项目的编译步骤按顺序逐一执行。
# 目前最低内存要求为16GB，如果需要在内存小于16GB的环境下生成，请在调用build.sh时指定-j参数限制并发操作数。
#
# 上述列出的步骤当中，同级别的步骤rebuild不会影响其他步骤，例如：
# 1. wr_ls_udp的文件更改只会导致build-wr_ls_udp步骤重新执行，其他步骤不受影响
# 2. capella_msgs发生改变会导致build-base及其下属所有步骤重新执行
# 3. 修改任意python包的内容只会重新执行build-python-packages或build-python-packages-nodeps，其他步骤不受影响
# 4. 在base中添加、删除packages.xml文件，或者修改package.xml文件的内容，会导致重新安装依赖项，并重新执行所有编译步骤
#
# 添加新的包时，请注意：
# 1. 必须在base步骤复制包的package.xml
# 2. 如果包不依赖capella_msgs，请继承build，然后添加到build-base之前执行
# 3. 如果包依赖capella_msgs，请继承build-base，然后添加到build-python-packages之前执行
# 4. 如果是python包且不依赖capella_msgs，将包加入build-python-packages-nodeps步骤即可
# 5. 如果是python包且依赖capella_msgs，将包加入build-python-packages步骤即可
# 6. 具有依赖关系的包（例如capella_ros_scan_odom->csm）应放在一个步骤中分开编译，参考build-capella_ros_scan_odom
# 7. 如果新加的包需要顺序编译，请在build-navigation2之后执行
FROM base AS build
ARG JOB_COUNT
WORKDIR /build

ENV MAKEFLAGS="-j${JOB_COUNT}"
RUN touch /BUILD_COMPLETE

# Build cartographer & cartographer_ros.
FROM build AS build-cartographer
COPY ["src/cartographer", "src/cartographer"]
RUN colcon build --base-paths src/cartographer --merge-install --parallel-workers 1 \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

COPY ["src/cartographer-ros/cartographer_ros", "src/cartographer-ros/cartographer_ros"]
RUN colcon build --base-paths src/cartographer-ros --merge-install --parallel-workers 1 \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

# Build ritju/navigation2 packages.
FROM build AS build-navigation2
ARG JOB_COUNT
COPY --from=build-cartographer ["/BUILD_COMPLETE","/"]
COPY ["src/navigation2/nav2_amcl", "src/navigation2/nav2_amcl"]
COPY ["src/navigation2/nav2_dwb_controller/dwb_critics", "src/navigation2/nav2_dwb_controller/dwb_critics"]
RUN colcon build --base-paths src/navigation2 --merge-install --parallel-workers ${JOB_COUNT} \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

# Build csm & scan odom
FROM build AS build-capella_ros_scan_odom
COPY --from=build-navigation2 ["/BUILD_COMPLETE","/"]
COPY ["src/csm", "src/csm"]
RUN . /opt/ros/humble/setup.sh && \
    colcon build --base-paths src/csm --merge-install --parallel-workers 1 \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic
COPY ["src/capella_ros_scan_odom", "src/capella_ros_scan_odom"]
RUN . /opt/ros/humble/setup.sh && \
    colcon build --base-paths src/capella_ros_scan_odom --merge-install --parallel-workers 1 \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

FROM build AS build-capella_ros_calib
COPY --from=build-navigation2 ["/BUILD_COMPLETE","/"]
COPY ["src/capella_ros_calib", "src/capella_ros_calib"]
RUN . /opt/ros/humble/setup.sh && \
    colcon build --base-paths src/capella_ros_calib --merge-install --parallel-workers 1 \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

# Build IMU filter package
FROM build AS build-capella_ros_imu_filter
COPY --from=build-navigation2 ["/BUILD_COMPLETE","/"]
COPY ["src/capella_ros_imu_filter", "src/capella_ros_imu_filter"]
RUN . /opt/ros/humble/setup.sh && \
    colcon build --base-paths src/capella_ros_imu_filter --merge-install --parallel-workers 1 \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

# Build lidar correction package
FROM build AS build-capella_ros_lidar_correction
COPY --from=build-navigation2 ["/BUILD_COMPLETE","/"]
COPY ["src/capella_ros_lidar_correction", "src/capella_ros_lidar_correction"]
RUN . /opt/ros/humble/setup.sh && \
    colcon build --base-paths src/capella_ros_lidar_correction --merge-install --parallel-workers 1 \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

FROM build AS build-wj_716n_lidar
COPY --from=build-navigation2 ["/BUILD_COMPLETE","/"]
COPY ["src/wj_716n_lidar", "src/wj_716n_lidar"]
RUN . /opt/ros/humble/setup.sh && \
    colcon build --base-paths src/wj_716n_lidar --merge-install --parallel-workers 1 \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

FROM build AS build-wr_ls_udp
COPY --from=build-navigation2 ["/BUILD_COMPLETE","/"]
COPY ["src/wr_ls_udp", "src/wr_ls_udp"]
RUN . /opt/ros/humble/setup.sh && \
    colcon build --base-paths src/wr_ls_udp --merge-install --parallel-workers 1 \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

FROM build AS build-wr_ls_udp_high
COPY --from=build-navigation2 ["/BUILD_COMPLETE","/"]
COPY ["src/wr_ls_udp_20_H025", "src/wr_ls_udp_20_H025"]
RUN . /opt/ros/humble/setup.sh && \
    colcon build --base-paths src/wr_ls_udp_20_H025 --merge-install --parallel-workers 1 \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

# Build ritju/capella_msgs packages.
FROM build AS build-base
COPY --from=build-navigation2 ["/BUILD_COMPLETE","/"]
COPY ["src/capella_msgs", "src/capella_msgs"]
RUN . /opt/ros/humble/setup.sh && \
    colcon build --base-paths src/capella_msgs --merge-install --parallel-workers 1 \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

# Build capella_ros_serial
FROM build-base AS build-capella_ros_serial
COPY ["src/capella_ros_serial", "src/capella_ros_serial"]
RUN . /opt/ros/humble/setup.sh && \
    colcon build --base-paths src/capella_ros_serial --merge-install --parallel-workers 1 \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

# Build capella_ros_auto_charge_ctrl
FROM build-base AS build-capella_ros_auto_charge_ctrl
COPY ["src/capella_ros_auto_charge_ctrl", "src/capella_ros_auto_charge_ctrl"]
RUN . /opt/ros/humble/setup.sh && \
    colcon build --base-paths src/capella_ros_auto_charge_ctrl --merge-install --parallel-workers 1 \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

# Build IMU calibration package
FROM build-base AS build-capella_ros_imu_calib
COPY ["src/capella_ros_imu_calib", "src/capella_ros_imu_calib"]
RUN . /opt/ros/humble/setup.sh && \
    colcon build --base-paths src/capella_ros_imu_calib --merge-install --parallel-workers 1 \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

# Build python packages.
FROM build-base AS build-python-packages
ARG JOB_COUNT
ENV MAKEFLAGS=""
COPY ["src/capella_charge_service", "src/capella_charge_service"]
COPY ["src/capella_nav_and_carto_server", "src/capella_nav_and_carto_server"]
COPY ["src/capella_ros_launcher", "src/capella_ros_launcher"]
RUN . /opt/ros/humble/setup.sh && \
    colcon build --base-paths \
      src/capella_charge_service \
      src/capella_nav_and_carto_server \
      src/capella_ros_launcher \
    --merge-install --parallel-workers ${JOB_COUNT} \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

FROM build AS build-python-packages-nodeps
ARG JOB_COUNT
ENV MAKEFLAGS=""
COPY ["src/capella_cartographer_launcher", "src/capella_cartographer_launcher"]
RUN . /opt/ros/humble/setup.sh && \
    colcon build --base-paths \
      src/capella_cartographer_launcher \
    --merge-install --parallel-workers ${JOB_COUNT} \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
    -Wall -Wextra -Wpedantic

# Produce final image based on the output of build stage.
FROM base AS final
RUN rm -rf /build

WORKDIR /capella

# External dependencies.
COPY --from=build-cartographer /build/install/ ./
COPY --from=build-navigation2 /build/install/ ./

# Local C++ packages with no dependency on capella_msgs.
COPY --from=build-capella_ros_scan_odom /build/install/ ./
COPY --from=build-capella_ros_calib /build/install/ ./
COPY --from=build-capella_ros_imu_filter /build/install/ ./
COPY --from=build-capella_ros_lidar_correction /build/install/ ./
COPY --from=build-wj_716n_lidar /build/install/ ./
COPY --from=build-wr_ls_udp /build/install/ ./
COPY --from=build-wr_ls_udp_high /build/install/ ./

# Local C++ packages which depdend on capella_msgs.
COPY --from=build-capella_ros_serial /build/install/ ./
COPY --from=build-capella_ros_auto_charge_ctrl /build/install/ ./
COPY --from=build-capella_ros_imu_calib /build/install/ ./

# Local Python packages.
COPY --from=build-python-packages-nodeps /build/install/ ./
COPY --from=build-python-packages /build/install/ ./

ARG VERSION=1.0.0
ENV IMAGE_VERSION="$VERSION"

ENV COLCON_CURRENT_PREFIX=/capella
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV DEBIAN_FRONTEND=

ENTRYPOINT . /capella/setup.sh && ros2 launch capella_nav_and_carto_server launch.xml