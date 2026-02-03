This project is to practice using ROS2

The main node and participant node will recieve, encrypt, decrypt and verify the message **once** before shutting down. 

Setup
1. clone the repo
git clone git@github.com:Maxiblast/av_ros2.git

3. build using
colcon build

4. change the params.yaml file to have your message and shift
   
5. use the launch file to run both the main node and participant node
ros2 launch av_ros2 cipher_launch.yaml

