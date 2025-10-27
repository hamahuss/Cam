Install ROS 2 Humble (Ubuntu 22.04)
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions


Source it:

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

2. Install PX4 ROS 2 message definitions

Create workspace:

mkdir -p ~/px4_ros_ws/src
cd ~/px4_ros_ws/src
git clone https://github.com/PX4/px4_msgs.git

3. Install the DDS Agent

PX4 uses Micro XRCE-DDS Agent for transport.

sudo apt install git build-essential cmake
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j4
sudo make install
sudo ldconfig


Now you can run:

MicroXRCEAgent serial --help

4. Create your custom publisher package (C++)

Inside your workspace:

cd ~/px4_ros_ws/src
mkdir -p vehicle_cam_pub_cpp/src
cd vehicle_cam_pub_cpp

🧾 package.xml
<?xml version="1.0"?>
<package format="3">
  <name>vehicle_cam_pub_cpp</name>
  <version>0.0.1</version>
  <description>ROS 2 C++ publisher for PX4 VehicleCamAttitude</description>

  <maintainer email="you@example.com">You</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>px4_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

⚙️ CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(vehicle_cam_pub_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(vehicle_cam_publisher src/vehicle_cam_publisher.cpp)
ament_target_dependencies(vehicle_cam_publisher rclcpp px4_msgs)

install(TARGETS
  vehicle_cam_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

💻 src/vehicle_cam_publisher.cpp
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "px4_msgs/msg/vehicle_cam_attitude.hpp"

using namespace std::chrono_literals;
using px4_msgs::msg::VehicleCamAttitude;

class VehicleCamPublisher : public rclcpp::Node
{
public:
    VehicleCamPublisher()
    : Node("vehicle_cam_publisher")
    {
        publisher_ = this->create_publisher<VehicleCamAttitude>("/fmu/in/vehicle_cam_attitude", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&VehicleCamPublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        VehicleCamAttitude msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000; // µs
        msg.q = {1.0f, 0.0f, 0.0f, 0.0f};

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Sent quaternion: [%.2f, %.2f, %.2f, %.2f]",
                    msg.q[0],
                    msg.q[1],
                    msg.q[2],
                    msg.q[3]);
    }

    rclcpp::Publisher<VehicleCamAttitude>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleCamPublisher>());
    rclcpp::shutdown();
    return 0;
}

5. Build everything
cd ~/px4_ros_ws
colcon build --symlink-install
source install/setup.bash


Add to ~/.bashrc for future sessions:

echo "source ~/px4_ros_ws/install/setup.bash" >> ~/.bashrc

6. Run and connect

Connect Pixhawk via USB or UART (adjust device name):

ls /dev/ttyACM*


Terminal 1 – DDS Agent

sudo MicroXRCEAgent serial -d /dev/ttyACM0 -b 921600


Terminal 2 – Publisher

source ~/px4_ros_ws/install/setup.bash
ros2 run vehicle_cam_pub_cpp vehicle_cam_publisher
