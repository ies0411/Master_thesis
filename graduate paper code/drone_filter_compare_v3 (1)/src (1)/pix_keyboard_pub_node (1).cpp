#ifndef __PIX_VM_KEYBOARD_PUB__
#define __PIX_VM_KEYBOARD_PUB__

#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

#include <string>
#include <thread>

#include "ros/ros.h"
const std::string Cmd_help = R"(
    1. takeoff (altitude velocity)
    2. landing    
    3. movebyvel x_vel y_vel z_vel yaw_rate
    4. movebypose x y z yaw
    5. waypoint x1 y1 z1 yaw1 x2 y2 z2 yaw2 ... finish
    6. manual
    7. markerlanding
    8. returntohome
    9. s  # stop
    10. scan2
    11. scan1
    12. pipe

    ****manual controll****
    q   w   e       r:go up
    a       d       f:go down
        s

)";
ros::Publisher keyboard_pub;
ros::Publisher watchdog_pub;

void WatchdogPub(void) {
    std_msgs::UInt32 watchdog_msg;
    watchdog_msg.data = 0;
    ros::Rate rate(10);
    while (ros::ok()) {
        watchdog_msg.data++;
        if (watchdog_msg.data > 100) {
            watchdog_msg.data = 0;
        }
        watchdog_pub.publish(watchdog_msg);
        rate.sleep();
    }
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_publisher");
    ros::NodeHandle nh;
    keyboard_pub = nh.advertise<std_msgs::String>("keyboard_command", 1);
    watchdog_pub = nh.advertise<std_msgs::UInt32>("watchdog_to_drone", 1);
    std_msgs::String msg;

    ros::Rate rate(10);
    std::thread thread_t2(WatchdogPub);
    thread_t2.detach();
    while (ros::ok()) {
        std::cout << Cmd_help << std::endl;
        std::cout << "command: ";
        std::getline(std::cin, msg.data);
        rate.sleep();
        keyboard_pub.publish(msg);
    }
    return 0;
}

#endif