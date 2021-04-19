#include "ros/ros.h"
#include "rospy_tutorials/AddTwoInts.h"

bool add(rospy_tutorials::AddTwoInts::Request  &req,
         rospy_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server"); //初始化node
  ros::NodeHandle n; // node handler
 
  ros::ServiceServer service = n.advertiseService("add_two_ints", add); //定義service server以及callback function
  ROS_INFO("Ready to add two ints.");
  ros::spin(); //持續運行此node

  return 0;
}