# Raspberry PI
sudo apt install git
# Package setup
rm -rf ~/ros_ws/src/usv_slam
cd ~/ros_ws/src
git clone https://github.com/Jagadeesh-pradhani/usv_slam.git

cd ~/ros_ws
colcon build --symlink-install
source ~/ros_ws/install/setup.bash

# serial device permission
echo "sudo chmod 777 /dev/ttyUSB0" >> ~/.bashrc
echo "sudo chmod 777 /dev/video0" >> ~/.bashrc

source ~/.bashrc

# running
# Terminal 1
cd ~/ros_ws
source ~/ros_ws/install/setup.bash
ros2 launch usv_slam mission.launch.py


---------------------------------------------------------------------------------------------
# Laptop
# Package setup

rm -rf ~/ros_ws/
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/Jagadeesh-pradhani/usv_slam.git
cd ~/ros_ws
colcon build
source ~/ros_ws/install/setup.bash

# running
# Terminal 1
cd ~/ros_ws
source ~/ros_ws/install/setup.bash
ros2 launch usv_slam visualization.launch.py