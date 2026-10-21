FROM ros:foxy-ros-base-focal
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-colcon-common-extensions \
    python3-numpy python3-opencv python3-pytest python3-yaml \
    ros-foxy-xacro ros-foxy-robot-state-publisher ros-foxy-rviz2 \
    ros-foxy-ament-cmake-gtest ros-foxy-ament-copyright \
    ros-foxy-ament-flake8 ros-foxy-ament-pep257 \
    && rm -rf /var/lib/apt/lists/*
ARG DEV_UID=1000
ARG DEV_GID=1000
RUN groupadd --gid "${DEV_GID}" developer \
    && useradd --uid "${DEV_UID}" --gid "${DEV_GID}" --create-home --shell /bin/bash developer \
    && mkdir -p /ws/build /ws/install /ws/log \
    && chown -R developer:developer /ws
COPY docker/foxy/entrypoint.sh /usr/local/bin/robominton-entrypoint
RUN chmod +x /usr/local/bin/robominton-entrypoint
ENV DEBIAN_FRONTEND=
USER developer
WORKDIR /ws
ENTRYPOINT ["/usr/local/bin/robominton-entrypoint"]
CMD ["bash"]
