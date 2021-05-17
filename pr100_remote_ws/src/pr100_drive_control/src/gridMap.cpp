
#include "ros/ros.h"
// #include "sensor_msgs/LaserScan.h"
// #include <sensor_msgs/Range.h>
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include<iomanip>


#define MAP_RES 0.1
#define MAP_W 200   // want if resolution = 1; MAP = 50*50 => map size = 50/RES
#define MAP_H 200


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
  map.info.origin.position.x = -(MAP_W*MAP_RES) / 2; //-10;
  map.info.origin.position.y = -(MAP_H*MAP_RES) / 2; //-10;
  
  int p[map.info.width*map.info.height] = {-1};   // [0,100]

  int cnt = 0;
  tf::TransformListener listener;
  ros::Rate rate(10.0); 
  while (ros::ok())
  {
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform("/grid", "/chassis",   
                                ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    
    std::cout << "get: " << transform.getOrigin().x() << ", " << transform.getOrigin().y() << "\n";

    double x = (transform.getOrigin().x() - map.info.origin.position.x)/MAP_RES;
    double y = (transform.getOrigin().y() - map.info.origin.position.y)/MAP_RES;
    
    std::cout << "trans: " << x << ", " << y << "\n";
    std::cout << "trans: " << round(x) << ", " << round(y) << "\n";
    
    x = round(x); y = round(y); 
    int pos = x + y*(double)MAP_H;
    // int pos = x*MAP_W + y;
    std::cout << pos << "\n";
    p[pos] = 100;        
    std::vector<signed char> a(p, p+MAP_W*MAP_H);
    map.data = a;
    pub.publish(map);


    // if (cnt <= MAP_W*MAP_H)
    // {
    //     p[cnt] = 100;   
    //     std::vector<signed char> a(p, p+MAP_W*MAP_H);
    //     map.data = a;
    //     pub.publish(map);
    // }
    // cnt++;


    // p[0] = 100;
    // p[500-1] = 100;   
    // p[1*500+1] = 100;   
    // p[499*500+0] = 100;  
    // p[499*500+249] = 100; 
    // std::vector<signed char> a(p, p+MAP_W*MAP_H);
    // map.data = a;
    // pub.publish(map);
    
    
    rate.sleep();
  }


  ros::shutdown();
  return 0;
}

