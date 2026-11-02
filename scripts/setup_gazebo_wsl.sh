#!/usr/bin/env bash
# ==============================================================================
# TurtleBot3 Gazebo 3D & ROS 2 Humble Automated Installer for Ubuntu 22.04 (WSL2)
# Robotics Laboratory ESE — Mecharithm Lesson 9
# ==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}======================================================================${NC}"
echo -e "${GREEN}     TURTLEBOT3 GAZEBO 3D + ROS 2 HUMBLE AUTOMATED SETUP (WSL2)      ${NC}"
echo -e "${BLUE}======================================================================${NC}"

# Check for Ubuntu
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$ID" != "ubuntu" ]; then
        echo -e "${YELLOW}[Warning] This script is optimized for Ubuntu. Detected: $NAME${NC}"
    fi
else
    echo -e "${RED}[Error] /etc/os-release not found. Make sure you are inside WSL2 Ubuntu.${NC}"
    exit 1
fi

echo -e "\n${BLUE}[1/5] Configuring system locale to UTF-8...${NC}"
sudo apt update && sudo apt install -y locales curl gnupg2 lsb-release
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo -e "\n${BLUE}[2/5] Adding official ROS 2 Humble Apt Repository & GPG Key...${NC}"
sudo mkdir -p /etc/apt/keyrings
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo -e "\n${BLUE}[3/5] Updating package index and installing ROS 2 & Gazebo packages...${NC}"
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3 \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pygame \
    python3-numpy \
    python3-yaml

echo -e "\n${BLUE}[4/5] Configuring ~/.bashrc environment variables...${NC}"

ensure_bashrc_entry() {
    local entry="$1"
    if ! grep -Fxq "$entry" ~/.bashrc; then
        echo "$entry" >> ~/.bashrc
    fi
}

ensure_bashrc_entry "# >>> TurtleBot3 & ROS 2 Humble Configuration >>>"
ensure_bashrc_entry "source /opt/ros/humble/setup.bash"
ensure_bashrc_entry "export TURTLEBOT3_MODEL=burger"
ensure_bashrc_entry "export ROS_DOMAIN_ID=0"
ensure_bashrc_entry "export ROS_LOCALHOST_ONLY=0"
ensure_bashrc_entry "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models"
ensure_bashrc_entry "# <<< TurtleBot3 & ROS 2 Humble Configuration <<<"

# Source for current shell session
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

echo -e "\n${BLUE}[5/5] Downloading TurtleBot3 Gazebo world models (avoids first-launch delay)...${NC}"
mkdir -p ~/.gazebo/models
if [ ! -d ~/.gazebo/models/turtlebot3_burger ]; then
    echo "Linking TurtleBot3 Gazebo models..."
    cp -r /opt/ros/humble/share/turtlebot3_gazebo/models/* ~/.gazebo/models/ 2>/dev/null || true
fi

echo -e "\n${GREEN}======================================================================${NC}"
echo -e "${GREEN}               GAZEBO 3D SETUP COMPLETED SUCCESSFULLY!                ${NC}"
echo -e "${GREEN}======================================================================${NC}"
echo -e "You can now run any of the following commands:"
echo -e "  1. Launch TurtleBot3 in 3D Gazebo World:"
echo -e "     ${YELLOW}ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py${NC}"
echo -e ""
echo -e "  2. Launch TurtleBot3 in Empty 3D World (fastest):"
echo -e "     ${YELLOW}ros2 launch turtlebot3_gazebo empty_world.launch.py${NC}"
echo -e ""
echo -e "  3. Drive with Keyboard Teleoperation:"
echo -e "     ${YELLOW}python3 scripts/teleoperation.py${NC}"
echo -e ""
echo -e "  4. Autonomous Navigation Controller:"
echo -e "     ${YELLOW}python3 scripts/gazebo_navigator.py${NC}"
echo -e "${GREEN}======================================================================${NC}"
