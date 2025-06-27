ARG BASE_IMAGE=ghcr.io/kas-lab/suave_rosa-headless:latest
FROM $BASE_IMAGE

RUN sudo apt update && sudo apt install -y \
    openjdk-17-jdk \
    gdb \
    && sudo rm -rf /var/lib/apt/list/

ENV JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64

COPY --chown=ubuntu-user:ubuntu-user config/ $HOME/suave_ws/src/suave_planta/config/
COPY --chown=ubuntu-user:ubuntu-user include/ $HOME/suave_ws/src/suave_planta/include/
COPY --chown=ubuntu-user:ubuntu-user launch/ $HOME/suave_ws/src/suave_planta/launch/
COPY --chown=ubuntu-user:ubuntu-user owl/ $HOME/suave_ws/src/suave_planta/owl/
COPY --chown=ubuntu-user:ubuntu-user pddl/ $HOME/suave_ws/src/suave_planta/pddl/
COPY --chown=ubuntu-user:ubuntu-user src/ $HOME/suave_ws/src/suave_planta/src/
COPY --chown=ubuntu-user:ubuntu-user CMakeLists.txt $HOME/suave_ws/src/suave_planta/CMakeLists.txt
COPY --chown=ubuntu-user:ubuntu-user package.xml $HOME/suave_ws/src/suave_planta/package.xml
COPY --chown=ubuntu-user:ubuntu-user suave_planta.repos $HOME/suave_ws/src/suave_planta/suave_planta.repos

WORKDIR $HOME/suave_ws
RUN vcs import src < $HOME/suave_ws/src/suave_planta/suave_planta.repos --force

RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && sudo apt update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y"]

# Build suave
RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install"]

RUN mkdir -p $HOME/suave/results
COPY --chown=ubuntu-user:ubuntu-user entrypoint.sh $HOME/suave_ws/entrypoint.sh

RUN sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/
ENTRYPOINT ["/home/ubuntu-user/suave_ws/entrypoint.sh"]
CMD [ "bash" ]