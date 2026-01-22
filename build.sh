#!/bin/bash

TARGET_PACKAGES=("openarm_description" "openarm_hardware" "openarm_bringup" "openarm_bimanual_moveit_config" "openarm_test")

# 执行 colcon 编译
colcon build \
    --symlink-install \
    --packages-select "${TARGET_PACKAGES[@]}" \