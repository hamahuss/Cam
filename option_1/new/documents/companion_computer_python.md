Companion computer setup (Jetson / Pi clean)
1️⃣ Install prerequisites
sudo apt update
sudo apt install -y git build-essential cmake python3-colcon-common-extensions python3-pip

2️⃣ Install ROS 2 Humble
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

3️⃣ Install Micro XRCE-DDS Agent (v2.x)
sudo apt install -y snapd
sudo snap install micro-xrce-dds-agent --edge
micro-xrce-dds-agent --version   # confirm v2.x

4️⃣ Create ROS 2 workspace
mkdir -p ~/px4_ros_ws/src
cd ~/px4_ros_ws/src

5️⃣ Get PX4 message definitions
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs
git checkout release/1.16 || true


Then replace/add your new message definition to match PX4 firmware:

cp ~/PX4-Autopilot/msg/VehicleCamAttitude.msg ~/px4_ros_ws/src/px4_msgs/msg/


Build the workspace:

cd ~/px4_ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

6️⃣ Connect Pixhawk and run Agent

Find your serial port (ls /dev/tty* → /dev/ttyACM0, /dev/ttyUSB0, or /dev/serial0).

Then run:

sudo micro-xrce-dds-agent serial -d /dev/ttyACM0 -b 921600


Keep it running — you’ll see logs once PX4 connects.

7️⃣ Create ROS 2 publisher node
cd ~/px4_ros_ws/src
ros2 pkg create --build-type ament_python vehicle_cam_pub --dependencies rclpy px4_msgs


Edit vehicle_cam_pub/vehicle_cam_pub/publisher_node.py:

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCamAttitude
import time
import numpy as np

class CamPub(Node):
    def __init__(self):
        super().__init__('cam_pub')
        self.pub = self.create_publisher(VehicleCamAttitude,
                                         '/fmu/in/vehicle_cam_attitude', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        msg = VehicleCamAttitude()
        msg.timestamp = int(time.time() * 1e6)
        # example quaternion (w, x, y, z)
        q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        msg.q = q.tolist()
        self.pub.publish(msg)
        self.get_logger().info(f'Published quaternion {msg.error_quaternion}')

def main(args=None):
    rclpy.init(args=args)
    node = CamPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


Make it executable:

chmod +x vehicle_cam_pub/vehicle_cam_pub/publisher_node.py


In vehicle_cam_pub/setup.py, ensure:

entry_points={
    'console_scripts': [
        'publisher_node = vehicle_cam_pub.publisher_node:main',
    ],
},

8️⃣ Build and run
cd ~/px4_ros_ws
colcon build --symlink-install
source install/setup.bash


Terminal 1:

sudo micro-xrce-dds-agent serial -d /dev/ttyACM0 -b 921600


Terminal 2:

ros2 run vehicle_cam_pub publisher_node

9️⃣ Verify

On companion:

ros2 topic list | grep vehicle_cam_attitude
# → /fmu/in/vehicle_cam_attitude
ros2 topic echo /fmu/in/vehicle_cam_attitude
