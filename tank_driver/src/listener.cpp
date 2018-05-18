#include "ros/ros.h"
#include "std_msgs/String.h"

void subCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "listener"); //初始化节点,名称为"listener"
    ros::NodeHandle nh;//创建节点句柄
    ros::Subscriber sub = nh.subscribe("chatter", 1000, subCallback); //创建订阅者
    ros::spin(); //阻塞调用
    return 0;
}
