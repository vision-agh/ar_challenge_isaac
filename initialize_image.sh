#!/bin/bash

cd /workspaces/ar_challenge_isaac
git config --global --add safe.directory "*"

echo 'export ISAACSIM_PATH=/isaac-sim/' >> /root/.bashrc 
echo 'alias ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"' >>  /root/.bashrc
echo 'alias ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"' >>  /root/.bashrc
echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/ar_challenge_isaac/fastdds.xml" >>  /root/.bashrc

rosdep init
rosdep update

# Install Pegasus Simulator
cd ./PegasusSimulator/extensions
/isaac-sim/python.sh -m pip install --editable pegasus.simulator
cd /workspaces/ar_challenge_isaac

# Install foonathan_memory_vendor
git clone https://github.com/eProsima/foonathan_memory_vendor.git
cd foonathan_memory_vendor
mkdir build && cd build
cmake ..
cmake --build . --target install

# Install PX4-Autopilot
cd ~/
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl

# Install Micro-XRCE-DDS-Agent
cd ~/
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

source /opt/ros/humble/setup.bash
mkdir /workspaces/px4_ros2_comm_ws
cd /workspaces/px4_ros2_comm_ws
mkdir src
cd src
git clone https://github.com/PX4/px4_ros_com
git clone https://github.com/PX4/px4_msgs.git
cd ..
colcon build

