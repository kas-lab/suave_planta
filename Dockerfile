FROM ghcr.io/kas-lab/suave_rosa-headless:latest

COPY --chown=ubuntu-user:ubuntu-user . $HOME/suave_ws/src/suave_planta/

WORKDIR $HOME/suave_ws
RUN vcs import src < $HOME/suave_ws/src/suave_planta/suave_planta.repos --force

RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && sudo apt update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y"]

# Build suave
RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install"]

RUN sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/