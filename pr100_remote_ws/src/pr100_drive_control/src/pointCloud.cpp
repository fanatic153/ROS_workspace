
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
 
int main (int argc, char **argv)
{
    ros::init (argc, argv, "pointCloud");
 
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud", 1);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;      // the "cloud" topic output msg
 
    // Fill in the cloud data
    cloud.width  = 100;  // total number of points in the cloud
    cloud.height = 1;   // total number of rows
    cloud.points.resize(cloud.width * cloud.height);  // stretched to 1D array
 
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
        // cloud.points[i].x = i;//1024 * rand () / (RAND_MAX + 1.0f);
        // cloud.points[i].y = i; //1024 * rand () / (RAND_MAX + 1.0f);
        // cloud.points[i].z = i; //1024 * rand () / (RAND_MAX + 1.0f);
        std::cout << cloud.points[i].x << "\n";
    }
 
    //Convert the cloud to ROS message
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "cloud";
 
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
 
    return 0;
}