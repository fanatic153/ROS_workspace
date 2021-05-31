
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
    cloud.width  = 5;  // total number of points in the cloud
    cloud.height = 1;   // total number of rows
    cloud.points.resize(cloud.width * cloud.height);  // stretched to 1D array
 
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        // cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        // cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        // cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].x = i;//1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].y = i; //1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].z = i; //1024 * rand () / (RAND_MAX + 1.0f);
        std::cout << cloud.points[i].x << "\n";
    }

    //////////////// 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr( new pcl::PointCloud<pcl::PointXYZ> (cloud) );
    // pcl::PointCloud<pcl::PointXYZ>::iterator index = cloudPtr->begin() + 2;
    // cloudPtr->erase(index); 

    //
    // for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloudPtr->begin(); it != cloudPtr->end(); it++) {
    //     if (it->z < 4.0) {
    //         cloudPtr->erase(it);
    //     }
    // }
    for(int nIndex = 0;nIndex < cloudPtr->points.size(); nIndex++)
    {
        cloudPtr->points[nIndex].x;
        cloudPtr->points[nIndex].y;
        cloudPtr->points[nIndex].z;

        if (cloudPtr->points[nIndex].x == 2.0) {
            pcl::PointCloud<pcl::PointXYZ>::iterator index = cloudPtr->begin() + nIndex;
            cloudPtr->erase(index); 
        }
    }

    

    /////////////

 
    //Convert the cloud to ROS message
    // pcl::toROSMsg(cloud, output);   // pub the original cloud
    pcl::toROSMsg(*cloudPtr, output);  // pub the pointer
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