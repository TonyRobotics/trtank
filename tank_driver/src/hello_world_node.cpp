#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "talker"); //初始化节点,名称为"talker"
    ros::NodeHandle nh; //创建节点句柄
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000); //创建发布者
    ros::Rate loop_rate(10); //设置循环频率
    int count = 0; //循环计数
    while (ros::ok()){
        std_msgs::String msg; //创建消息
        std::stringstream ss;
        ss << "Hello " << count;
        msg.data = ss.str(); //消息赋值
        ROS_INFO("send [%s]", msg.data.c_str());
        
        pub.publish(msg); //发布消息
        ros::spinOnce(); //非阻塞调用
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
