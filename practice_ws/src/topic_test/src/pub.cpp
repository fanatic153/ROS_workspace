#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
    /* init node "pub" */
    ros::init(argc, argv, "cpp_pub_node");

    ros::NodeHandle n;
    
    ros::Publisher pub = n.advertise<std_msgs::String>("try_topic", 1000);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        std_msgs::String msg;
        msg.data = "c++ pub";

        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}