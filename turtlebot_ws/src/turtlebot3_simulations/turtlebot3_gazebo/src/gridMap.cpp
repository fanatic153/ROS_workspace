
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#define MAP_RES 0.1
#define MAP_W 500   // want if resolution = 1; MAP = 50*50 => map size = 50/RES
#define MAP_H 500



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
  map.info.origin.position.x = -25; //-(MAP_W/MAP_RES) / 2; //-10;
  map.info.origin.position.y = -25; //-MAP_H / 2; //-10;
  
  int p[map.info.width*map.info.height] = {-1};   // [0,100]

  int cnt = 0;
  tf::TransformListener listener;
  ros::Rate rate(10.0); 
  while (ros::ok())
  {
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform("/grid", "/base_footprint",   
                                ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    
    std::cout << "get: " << transform.getOrigin().x() << ", " << transform.getOrigin().y() << "\n";
       
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();

    double x = (transform.getOrigin().x() - map.info.origin.position.x)/MAP_RES;
    double y = (transform.getOrigin().y() - map.info.origin.position.y)/MAP_RES;
    
    std::cout << "trans: " << x << ", " << y << "\n";

    int pos = x + y*(double)MAP_H;
    // int pos = x*MAP_W + y;
    std::cout << pos << "\n";
    // p[pos] = 100;        
    // std::vector<signed char> a(p, p+MAP_W*MAP_H);
    // map.data = a;
    // pub.publish(map);


    // if (cnt <= MAP_W*MAP_H)
    // {
    //     p[cnt] = 100;   
    //     std::vector<signed char> a(p, p+MAP_W*MAP_H);
    //     map.data = a;
    //     pub.publish(map);
    // }
    // cnt++;
    // p[0] = 100;   
    // p[250-1] = 100;   
    // p[500-1] = 100;   

    // p[1*500+1] = 100;   
    // p[499*500+0] = 100;  
    // p[499*500+249] = 100;   
    // p[110260] = 100;  
    p[122569] = 100;  
    p[122732] = 100;  
    std::vector<signed char> a(p, p+MAP_W*MAP_H);
    map.data = a;
    pub.publish(map);
    
    
    rate.sleep();
  }

//   while (ros::ok())
//   {
//       pub.publish(map);
//   }

  ros::shutdown();
  return 0;
}



















// #include "ros/ros.h"
// // #include "sensor_msgs/LaserScan.h"
// #include "nav_msgs/OccupancyGrid.h"
// #include <tf/transform_listener.h>
// #include <tf/transform_broadcaster.h>
// #include <nav_msgs/Odometry.h>


// #define MAP_W     50
// #define MAP_H     50
// #define MAP_RES   0.01

// bool update = false;
// double pos_x, pos_y;

// void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
// {
//   // double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
//   // double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

//   // tb3_pose_ = atan2(siny, cosy);

//   std::cout << "position = " << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << "\n";
//   pos_x = msg->pose.pose.position.x;
//   pos_y = msg->pose.pose.position.y; 
//   // updateMap(msg->pose.pose.position.x, msg->pose.pose.position.y);
//   update = true;
// }



// int main(int argc, char * argv[]) 
// {  
//   ros::init(argc, argv, "gridMap");  // init node

//   ros::NodeHandle nh;
//   ros::NodeHandle nh_private("~");

//   ros::Subscriber odom_sub_ = nh.subscribe("odom", 10, odomMsgCallBack);

//   ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/gridMap", 1);
//   nav_msgs::OccupancyGrid map;

//   map.header.frame_id = "grid";
//   map.header.stamp = ros::Time::now(); 
//   map.info.resolution = MAP_RES;         // float32
//   map.info.width      = MAP_W;           // uint32
//   map.info.height     = MAP_H;           // uint32
//   // map.info.origin.position.x = 0.0;
//   // map.info.origin.position.y = 0.0;
//   // map.info.origin.position.z = 0.0;

//   int p[map.info.width*map.info.height] = {-1}; // value of prob: [0,100]

  
//   ros::Rate loop_rate(10);

//   tf::TransformListener listener;
//   tf::StampedTransform transform;

//   while (ros::ok())
//   {
//     // try
//     // {
//     //   listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);  // parent, child
//     // }
//     // catch (tf::TransformException ex)
//     // {
//     //   ROS_ERROR("%s",ex.what());
//     //   ros::Duration(1.0).sleep();
//     // }
    
//     std::cout << update << "\n";
//     if (update == true)
//     {
//       // fill map
//       int32_t x = (pos_x/ MAP_RES);
//       if (pos_x > 0)
//         x = x + 1;
//       else if (pos_x < 0)
//         x = x - 1;
//       std::cout << "x = " << x << "\n";
//       int32_t y = (pos_y / MAP_RES);
//       if (pos_y > 0)
//         y = y + 1;
//       else if (pos_y < 0)
//         y = y - 1;
//       std::cout << "y = " << y << "\n";

//       std::cout << "cnt = " << (x*MAP_W + y) << "\n";
//       p[x*MAP_W + y] = 100;   // mark this cell
//       std::cout << "p = " << p[x*MAP_W + y] << "\n";

//       std::vector<signed char> a(p, p+MAP_W*MAP_H);    // map.info.width*map.info.height
//       map.data = a;
      
//       map_pub.publish(map);

//       update = false;
//     }
    
//     ros::spinOnce();
//     loop_rate.sleep();
//   }

//   ros::shutdown();
//   return 0;
// }