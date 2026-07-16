FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-serial \
        python3-shapely \
        python3-yaml \
        ros-humble-nmea-navsat-driver \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/mower_ws

# The root .dockerignore keeps generated build/install/log artifacts out of
# the context.  Only the two hardware bring-up packages are built here; the
# simulator and FAST-LIO are intentionally not part of this image.
COPY src /opt/mower_ws/src

RUN source /opt/ros/humble/setup.bash \
    && colcon build --merge-install \
        --packages-select mower_coverage mower_hardware

COPY deploy/rk3588/entrypoint.sh /usr/local/bin/mower-rk3588-entrypoint
RUN chmod 0755 /usr/local/bin/mower-rk3588-entrypoint

ENTRYPOINT ["/usr/local/bin/mower-rk3588-entrypoint"]
CMD ["ros2", "launch", "mower_hardware", "rtk_readonly.launch.py"]
