#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
	/* init node "listener" */
  ros::init(argc, argv, "cpp_sub_node");

  /* NodeHandle is the main access point to communications with the ROS system. */
  ros::NodeHandle n;

	/* subscribe: topic "chatter"; Have a callback function dealing with incomming message  */
  ros::Subscriber sub = n.subscribe("try_topic", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}