#!/bin/bash
# run_all.sh - Launch the complete GNSS-Denied Drone Navigation System stack

echo "🚁 Starting GPS-Denied Drone Navigation System..."

WORKSPACE_DIR=$(pwd)
source $WORKSPACE_DIR/install/setup.bash

# Ensure Webots server node or display is ready 
# You might want to start Xvfb or ensure DISPLAY is set for Webots

# Start Master Launch
ros2 launch src/launch/master_launch.py
