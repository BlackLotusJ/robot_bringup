#!/bin/bash

# Script to publish robot description and necessary transforms
# Run this alongside your other robot nodes

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}🤖 Publishing Robot Description and Transforms...${NC}"

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
URDF_FILE="$SCRIPT_DIR/rocker_bogie_rover_with_lidar.urdf.xacro"

# Check if URDF file exists
if [ ! -f "$URDF_FILE" ]; then
    echo -e "${RED}❌ Error: URDF file not found at: $URDF_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Found URDF file: $URDF_FILE${NC}"

# Function to run command in background and store PID
run_in_background() {
    local cmd="$1"
    local name="$2"
    echo -e "${YELLOW}🚀 Starting: $name${NC}"
    eval "$cmd" &
    local pid=$!
    echo "$pid" >> /tmp/robot_description_pids.txt
    echo -e "${GREEN}✅ Started $name (PID: $pid)${NC}"
}

# Clean up previous PIDs file
rm -f /tmp/robot_description_pids.txt

echo ""
echo -e "${YELLOW}📡 Starting robot description publisher...${NC}"

# Start robot state publisher
run_in_background "ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"\$(xacro $URDF_FILE)\"" "Robot State Publisher"

# Start joint state publisher (for the wheels)
run_in_background "ros2 run joint_state_publisher joint_state_publisher" "Joint State Publisher"

# Static transform from base_footprint to base_link (if needed)
run_in_background "ros2 run tf2_ros static_transform_publisher 0 0 0.06 0 0 0 base_footprint base_link" "Base Footprint Transform"

# Static transform for laser frame (this should match your URDF, but adding for safety)
run_in_background "ros2 run tf2_ros static_transform_publisher 0.36 0 0.17 0 0 0 base_link laser_frame" "Laser Frame Transform"

# Static transform for IMU
run_in_background "ros2 run tf2_ros static_transform_publisher 0 0 0.11 0 0 0 base_link imu_link" "IMU Frame Transform"

echo ""
echo -e "${GREEN}🎉 All robot description nodes started!${NC}"
echo -e "${YELLOW}📋 Running nodes:${NC}"
echo -e "${YELLOW}   - Robot State Publisher${NC}"
echo -e "${YELLOW}   - Joint State Publisher${NC}"
echo -e "${YELLOW}   - Static Transform Publishers${NC}"
echo ""
echo -e "${YELLOW}💡 To stop all nodes, run:${NC}"
echo -e "${YELLOW}   kill \$(cat /tmp/robot_description_pids.txt)${NC}"
echo ""
echo -e "${GREEN}✅ You can now launch RViz and see your robot model!${NC}"

# Keep script running and handle Ctrl+C
cleanup() {
    echo ""
    echo -e "${YELLOW}🛑 Stopping all robot description nodes...${NC}"
    if [ -f /tmp/robot_description_pids.txt ]; then
        while IFS= read -r pid; do
            if kill -0 "$pid" 2>/dev/null; then
                echo -e "${YELLOW}🔌 Killing process $pid${NC}"
                kill "$pid"
            fi
        done < /tmp/robot_description_pids.txt
        rm -f /tmp/robot_description_pids.txt
    fi
    echo -e "${GREEN}👋 Robot description publisher stopped!${NC}"
    exit 0
}

trap cleanup SIGINT

# Wait for processes to finish
wait
