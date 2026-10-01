#!/bin/bash

# 1. Source the ROS 2 workspace
source /home/er4-user/ws/install/setup.bash

echo "=========================================="
echo "🌕 Starting Togo LUNAR Simulation..."
echo "=========================================="

# 2. Start the Gazebo Server (Headless/GPU for LiDAR) in the background
echo "[1/3] Starting Gazebo Server (GPU)..."
GZ_SIM_RESOURCE_PATH=/home/er4-user/ws/install/togo_gz/share/togo_gz:/opt/ros/jazzy/share \
DISPLAY= ros2 launch togo_gz sim_gz.launch.py \
    world:=lunar_surface.world \
    robot_x:=1.0 \
    robot_y:=12.0 \
    robot_z:=1.0 &


SERVER_PID=$!

# Lunar heightmap takes longer to load than the parking lot world
sleep 8

# 3. Start the Gazebo GUI (Software Rendering for ThinLinc) in the background
echo "[2/3] Starting Gazebo GUI (CPU)..."
LIBGL_ALWAYS_SOFTWARE=1 gz sim -g &
GUI_PID=$!

sleep 2

# 4. Start RViz2 in the background
echo "[3/3] Starting RViz2..."
#rviz2 -d /home/er4-user/ws/install/togo_deploy/share/togo_deploy/rviz/#robot_sensor_checkout.rviz &
#RVIZ_PID=$!

echo "=========================================="
echo "✅ All systems running! Press Ctrl+C to exit."
echo "=========================================="

# 5. Cleanup function
cleanup() {
    echo -e '\n🛑 Shutting down all processes...'
    kill $SERVER_PID $GUI_PID $RVIZ_PID 2>/dev/null
    killall -9 gz          2>/dev/null
    killall -9 gz-sim      2>/dev/null
    killall -9 gzserver    2>/dev/null
    killall -9 gzclient    2>/dev/null
    wait
    echo "✅ All processes terminated."
    exit
}

# 6. Trap Ctrl+C (SIGINT) and SIGTERM to run cleanup
trap cleanup SIGINT SIGTERM

wait
