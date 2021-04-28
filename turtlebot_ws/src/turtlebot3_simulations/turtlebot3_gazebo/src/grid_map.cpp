#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>

#define MAP_W 40
#define MAP_H 40
#define MAP_RES 0.5


int main(int argc, char * argv[]) {

  ros::init(argc, argv, "gridMap");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/gridMap", 1);
  nav_msgs::OccupancyGrid map;

  map.header.frame_id = "grid";
  map.header.stamp = ros::Time::now(); 
  map.info.resolution = MAP_RES;         // float32
  map.info.width      = MAP_W;           // uint32
  map.info.height     = MAP_H;           // uint32
  map.info.origin.position.x = 0;//-MAP_W / 2; //-10;
  map.info.origin.position.y = 0;//-MAP_H / 2; //-10;
  
  int p[map.info.width*map.info.height] = {-1};   // [0,100]
  

  tf::TransformListener listener;
  ros::Rate rate(10.0); int cnt = 0;
  while (ros::ok())
  {
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform("/base_footprint", "/odom",  
                                ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    
    std::cout << "get: " << transform.getOrigin().x() << ", " << transform.getOrigin().y() << "\n";
    
    // int x = transform.getOrigin().x(); // - map.info.origin.position.x;
    // if (x > 0)
    //     x = (transform.getOrigin().x() + 1 ) - map.info.origin.position.x;
    // else if (x < 0)
    //     x = (transform.getOrigin().x() - 1 ) - map.info.origin.position.x;

    // int y = transform.getOrigin().y(); // - map.info.origin.position.x;
    // if (y > 0)
    //     y = (transform.getOrigin().x() + 1 ) - map.info.origin.position.y;
    // else if (y < 0)
    //     y = (transform.getOrigin().x() - 1 ) - map.info.origin.position.y;
    
    int x = transform.getOrigin().x()/MAP_RES + MAP_W/2;//- map.info.origin.position.x;
    int y = transform.getOrigin().y()/MAP_RES + MAP_H/2; //- map.info.origin.position.x;
    std::cout << "trans: " << x << ", " << y << "\n";
    int pos = x + y*MAP_H;
    p[pos] = 100;    std::cout << pos << "\n";
    std::vector<signed char> a(p, p+MAP_W*MAP_H);
    map.data = a;
    pub.publish(map);


    // if (cnt <= MAP_W*MAP_H)
    // {
    //     p[cnt] = 100;    std::cout << pos << "\n";
    //     std::vector<signed char> a(p, p+MAP_W*MAP_H);
    //     map.data = a;
    //     pub.publish(map);
    // }
    // cnt++;
    
    
    
    rate.sleep();
  }

//   while (ros::ok())
//   {
//       pub.publish(map);
//   }

  ros::shutdown();
  return 0;
}