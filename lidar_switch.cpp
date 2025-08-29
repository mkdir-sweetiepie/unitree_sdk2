#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/common/thread/thread.hpp>
#include <iostream>
#include <string>
#include <unistd.h>

using namespace unitree::common;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <network_interface> [ON|OFF]" << std::endl;
        return 1;
    }
    
    std::string network_interface = argv[1];
    std::string command = (argc > 2) ? argv[2] : "ON";
    
    std::cout << "Turning lidar " << command << " via network interface: " << network_interface << std::endl;
    
    try {
        // Create publisher for lidar switch
        unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_> publisher("rt/utlidar/switch");
        publisher.InitChannel();
        
        // Create message
        std_msgs::msg::dds_::String_ message;
        message.data() = command;
        
        // Publish the command multiple times to ensure delivery
        for (int i = 0; i < 5; i++) {
            publisher.Write(message);
            sleep(1);
        }
        
        std::cout << "Lidar switch command sent: " << command << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}