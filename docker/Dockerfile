# Bemu development image: ROS 2 Lyrical + MuJoCo + the CAD tools, for WSL 2 or any Linux with Docker.
#
#   docker/dev build        (or: docker compose -f docker/compose.yaml build)
#
# Not yet built on the owner's PC; CI builds it on every push (the `docker` job).

# Stage 1: keep only the package manifests, so the dependency layer is rebuilt only when a
# dependency changes, not on every source edit.
FROM ros:lyrical-ros-base AS manifests
COPY ros2_ws/src /tmp/src
RUN find /tmp/src -type f ! -name package.xml -delete \
 && find /tmp/src -type d -empty -delete

# Stage 2: the development image.
FROM ros:lyrical-ros-base
ARG USERNAME=bemu
ARG UID=1000
ARG GID=1000
ENV DEBIAN_FRONTEND=noninteractive \
    BEMU_PARAMS=/workspace/bemu/config/robot_params.yaml \
    PIP_BREAK_SYSTEM_PACKAGES=1

# System packages: the control stack and the MuJoCo bridge (docs/13), RViz, 3D drivers for WSLg,
# and the tools the scripts use.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-lyrical-ros2-control ros-lyrical-ros2-controllers \
      ros-lyrical-mujoco-ros2-control ros-lyrical-mujoco-ros2-control-demos \
      ros-lyrical-rviz2 \
      python3-pip python3-pytest python3-numpy python3-yaml python3-matplotlib \
      mesa-utils libgl1-mesa-dri libglu1-mesa x11-apps \
      git sudo less nano ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# ROS dependencies of the packages in the workspace (the same step CI runs).
COPY --from=manifests /tmp/src /tmp/rosdep_src
RUN apt-get update \
 && rosdep update --rosdistro lyrical \
 && rosdep install --from-paths /tmp/rosdep_src -y --ignore-src \
 && rm -rf /var/lib/apt/lists/* /tmp/rosdep_src

# MuJoCo for the system Python (ROS side), as in CI.
RUN pip install mujoco

# CAD tools: build123d needs a Python it has wheels for, so it gets its own Python 3.13
# virtualenv at /opt/cad. `/opt/cad/bin/python` runs the same pytest as the CI "python" job.
COPY requirements-dev.txt /tmp/requirements-dev.txt
RUN pip install uv \
 && uv venv --python 3.13 /opt/cad \
 && uv pip install --python /opt/cad/bin/python -r /tmp/requirements-dev.txt matplotlib \
 && rm /tmp/requirements-dev.txt

# A user with the host's UID/GID, so files written into the mounted repository stay yours.
RUN (getent passwd "${UID}" | cut -d: -f1 | xargs -r userdel -r) 2>/dev/null || true \
 && (getent group "${GID}" >/dev/null || groupadd -g "${GID}" "${USERNAME}") \
 && useradd -m -u "${UID}" -g "${GID}" -s /bin/bash "${USERNAME}" \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && mkdir -p /workspace/bemu/ros2_ws/build /workspace/bemu/ros2_ws/install /workspace/bemu/ros2_ws/log \
 && chown -R "${UID}:${GID}" /workspace

COPY docker/entrypoint.sh /usr/local/bin/bemu-entrypoint
RUN chmod +x /usr/local/bin/bemu-entrypoint \
 && printf '%s\n' \
      'source /opt/ros/lyrical/setup.bash' \
      '[ -f /workspace/bemu/ros2_ws/install/setup.bash ] && source /workspace/bemu/ros2_ws/install/setup.bash' \
      'alias cadpy=/opt/cad/bin/python' \
      >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /workspace/bemu
ENTRYPOINT ["/usr/local/bin/bemu-entrypoint"]
CMD ["bash"]
