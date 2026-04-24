#!/bin/bash

set -e  # 하나라도 실패하면 바로 종료

echo "===== Step 1: Clone quadruped_ros2_control ====="
cd ~/ros2_ws/src
git clone -b run_v1 https://github.com/whaudrms/quadruped_ros2_control.git

echo "===== Step 2: Install dependencies ====="
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

echo "===== Step 3: Build base packages ====="
colcon build --packages-up-to unitree_guide_controller go2_description keyboard_input --symlink-install
colcon build --packages-up-to hardware_unitree_sdk2

echo "===== Step 4: Clone ocs2_ros2 ====="
cd ~/ros2_ws/src
git clone https://github.com/legubiao/ocs2_ros2

echo "===== Step 5: Init submodules ====="
cd ~/ros2_ws/src/ocs2_ros2
git submodule update --init --recursive

echo "===== Step 6: Install dependencies again ====="
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

echo "===== Step 7: Build ocs2 controller (1st pass) ====="
colcon build --packages-up-to ocs2_quadruped_controller \
    --symlink-install \
    --parallel-workers 1

echo "===== Step 8: Build ocs2 controller (Debug patch build) ====="
colcon build --packages-up-to ocs2_quadruped_controller \
    --base-paths ~/ros2_ws/src/quadruped_ros2_control \
    --build-base /tmp/perceptive_v2_build_patch \
    --install-base /tmp/perceptive_v2_install_patch \
    --symlink-install \
    --executor sequential \
    --parallel-workers 1 \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_CXX_FLAGS=-O0 \
        -DCMAKE_C_FLAGS=-O0

echo "===== DONE ====="
