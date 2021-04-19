
/*
#include <ros/ros.h>                             // 引用 ros.h 檔

int main(int argc, char** argv){
    ros::init(argc, argv, "hello_cpp_node");     // 初始化 hello_cpp_node
    ros::NodeHandle handler;                     // node 的 handler
    ROS_INFO("Hello World!");                    // 印出 Hello World
}
*/

#include <ros/ros.h>                             // 引用 ros.h 檔

int main(int argc, char** argv){
    ros::init(argc, argv, "hello_cpp_node");     // 初始化hello_cpp_node
    ros::NodeHandle handler;                     // node 的 handler
    while (ros::ok()){                           // 在 ros 順利執行時
        ROS_INFO("Hello World!");                // 印出 Hello World
        ros::Duration(1).sleep();                // 間隔 1 秒
    }
}
