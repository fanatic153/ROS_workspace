
#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <time.h> 

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.1
#define ANGULAR_VELOCITY 1.5

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3

#define DISTANCE_THRESH 0.3//0.5

class Pr100Drive
{
 public:

    Pr100Drive();
    ~Pr100Drive();

    void init(void);
    void controlLoop(void);
    void showSensorData(void);

 private:

    // Variables
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Subscriber laser_sub;
    ros::Subscriber sonar1_sub;
    ros::Subscriber sonar2_sub;
    ros::Publisher cmd_vel_pub;

    double pose;
    double scan_data_[3] = {0.0, 0.0, 0.0};
    double range1, range2;

    void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
    void sonar1MsgCallBack(const sensor_msgs::Range::ConstPtr &msg);
    void sonar2MsgCallBack(const sensor_msgs::Range::ConstPtr &msg);
    void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
    void updatecommandVelocity(double linear, double angular);
    
};

Pr100Drive::Pr100Drive()// : nh_priv_("~")
{
    //Init ros pr100 node
    ROS_INFO("pr100 Simulation Node Init");
}

Pr100Drive::~Pr100Drive()
{        
    ROS_INFO("pr100 Simulation Node Shutdown");
    ros::shutdown();        
}

void Pr100Drive::init(void)
{    
    // initialize subscriber
    laser_sub = nh.subscribe("/pr100/laser/scan", 10, &Pr100Drive::laserScanMsgCallBack, this);
    
    sonar1_sub = nh.subscribe("/pr100/sonar1", 10, &Pr100Drive::sonar1MsgCallBack, this);
    sonar2_sub = nh.subscribe("/pr100/sonar2", 10, &Pr100Drive::sonar2MsgCallBack, this);
    odom_sub = nh.subscribe("odom", 10, &Pr100Drive::odomMsgCallBack, this);

    // initialize publishers
    cmd_vel_pub   = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
}

void Pr100Drive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)    
{
    // Lidar have 360 degree of data, we take only three value
    uint16_t scan_angle[3] = {0, 30, 330};
    
    // std::cout << "lasec scan = \n";

    for (int num = 0; num < 3; num++)
    {
        if (std::isinf(msg->ranges.at(scan_angle[num])))
        {
            scan_data_[num] = msg->range_max;
            // std::cout << "  " << scan_data_[num] << "\n";
        }
        else
        {
            scan_data_[num] = msg->ranges.at(scan_angle[num]);
            // std::cout << "  " << scan_data_[num] << "\n";
        }
    }
}

void Pr100Drive::sonar1MsgCallBack(const sensor_msgs::Range::ConstPtr &msg)
{
    range1 = msg->range;
}

void Pr100Drive::sonar2MsgCallBack(const sensor_msgs::Range::ConstPtr &msg)
{
    range2 = msg->range;
}

void Pr100Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

    pose = atan2(siny, cosy);

    // std::cout << "pose = " << pose << "\n";        
}

void Pr100Drive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub.publish(cmd_vel);
}

// void Pr100Drive::controlLoop(void)
// {
//     if (scan_data_[LEFT] < DISTANCE_THRESH || scan_data_[CENTER] < DISTANCE_THRESH || scan_data_[RIGHT] < DISTANCE_THRESH)
//     {
//         updatecommandVelocity(-0.05 , ANGULAR_VELOCITY);
//         // std::cout << "Turn!\n";
//     }
//     else
//     {
//         updatecommandVelocity(LINEAR_VELOCITY , 0.0);    
//         // std::cout << "GO!\n";
//     }
// }

void Pr100Drive::controlLoop(void)
{
    if (range1 < DISTANCE_THRESH || range1 < DISTANCE_THRESH)
    {
        updatecommandVelocity(-0.05 , ANGULAR_VELOCITY);
        // std::cout << "Turn!\n";
    }
    else
    {
        updatecommandVelocity(LINEAR_VELOCITY , 0.0);    
        // std::cout << "GO!\n";
    }
}

void Pr100Drive::showSensorData(void)
{
    std::cout << "scan data = \n";
    std::cout << "  " << scan_data_[LEFT] << "\n";
    std::cout << "  " << scan_data_[CENTER] << "\n";
    std::cout << "  " << scan_data_[RIGHT] << "\n";

    std::cout << "sonar1, sonar2 = \n"; 
    std::cout << "  " << range1 << "\n";
    std::cout << "  " << range2 << "\n";

    std::cout << "pose, prev = \n"; 
    std::cout << "  " << pose << "\n";
    // std::cout << "  " << prev_tb3_pose_ << "\n";
    std::cout << "\n";
}

/* ----- main ------- */
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pr100_drive");
    
    Pr100Drive *pr100_drive = new Pr100Drive();

    pr100_drive->init();

    ros::Rate loop_rate(125);

    srand( time(NULL) );
    int count = 0;
    while (ros::ok())
    {    
        pr100_drive->controlLoop();

        count++;
        if (count >= 100)
        {
            pr100_drive->showSensorData();
            count = 0;
        }        

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
