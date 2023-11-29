#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
source /app/install/setup.bash
echo "Provided arguments: $@"

exec $@