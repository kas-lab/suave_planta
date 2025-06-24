#!/bin/bash
set -e
source "/opt/ros/humble/setup.bash"
chown -R ubuntu-user:ubuntu-user /home/ubuntu-user/suave/results

exec "$@"