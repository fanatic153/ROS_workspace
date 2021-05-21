
#include <ros/ros.h>

#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <time.h> 
#include <unistd.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.1
#define ANGULAR_VELOCITY 0.2

#define CLOCKWISE 1
#define COUNTER_CLOCKWISE -1

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3

#define DISTANCE_THRESH 0.29 //0.32 //0.42 //0.5

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
    ros::Subscriber sonar1_sub;
    ros::Subscriber sonar2_sub;
    ros::Publisher cmd_vel_pub;

    double pose;
    double range1, range2;

    clock_t clk_mark;
    time_t t_mark;

    void sonar1MsgCallBack(const sensor_msgs::Range::ConstPtr &msg);
    void sonar2MsgCallBack(const sensor_msgs::Range::ConstPtr &msg);
    void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
    void updatecommandVelocity(double linear, double angular);

    void turn_theta(double theta, int direction);
    void go_forward(void);
    void go_backward(void);
    void stop(void);
    
};

Pr100Drive::Pr100Drive()// : nh_priv_("~")
{
    //Init ros pr100 node
    ROS_INFO("pr100 Simulation Node Init");
}

Pr100Drive::~Pr100Drive()
{        
    updatecommandVelocity(0.0 , 0.0);
    ROS_INFO("pr100 Simulation Node Shutdown");
    ros::shutdown();        
}

void Pr100Drive::init(void)
{    
    // initialize subscriber 
    sonar1_sub = nh.subscribe("/pr100/sonar1", 10, &Pr100Drive::sonar1MsgCallBack, this);
    sonar2_sub = nh.subscribe("/pr100/sonar2", 10, &Pr100Drive::sonar2MsgCallBack, this);
    odom_sub = nh.subscribe("odom", 10, &Pr100Drive::odomMsgCallBack, this);

    // initialize publishers
    cmd_vel_pub   = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
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

void Pr100Drive::turn_theta(double theta, int direction)
{
    // double t_turn = (theta * DEG2RAD) / ANGULAR_VELOCITY;
    unsigned int t_turn = 100000 * ( (theta * DEG2RAD) / ANGULAR_VELOCITY );  // 100000
    std::cout << "t_turn = " << t_turn; // << "\n";    

    t_mark = time(NULL);

    updatecommandVelocity(0.0, direction * ANGULAR_VELOCITY);
    usleep(t_turn);    // sleep in micro-sec
    updatecommandVelocity(0.0, 0.0);

    std::cout << " time elapsed: " << time(NULL) - t_mark << "\n";
    
}

void Pr100Drive::go_forward(void)
{
    updatecommandVelocity(LINEAR_VELOCITY , 0.0);
}

void Pr100Drive::go_backward(void)
{
    updatecommandVelocity((-1) * LINEAR_VELOCITY , 0.0);
}

void Pr100Drive::stop(void)
{
    updatecommandVelocity(0.0 , 0.0);  
}

void Pr100Drive::controlLoop(void)
{

    std::cout << "one\n";
    turn_theta(180, CLOCKWISE);
    usleep(1000000);
    std::cout << "two\n";
    // updatecommandVelocity(LINEAR_VELOCITY , 0.0);
    go_forward();
    usleep(1000000);
    std::cout << "stop\n\n";
    stop();
    usleep(1000000);

    // double theta = 30;

    // if (range1 < 0.2 && range2 < 0.2)
    // {
    //     go_backward();
    // }
    // else if (range1 < DISTANCE_THRESH || range2 < DISTANCE_THRESH)
    // {
    //     if (range1 > range2 + 0.05)
    //     {
    //         turn_theta(theta, CLOCKWISE);
    //     }
    //     else
    //     {
    //         turn_theta(theta, COUNTER_CLOCKWISE);             
    //     }
        
    //     // updatecommandVelocity(0.0 , ANGULAR_VELOCITY);
    //     // std::cout << "Turn!\n";
    // }
    // else
    // {
    //     updatecommandVelocity(LINEAR_VELOCITY , 0.0);    
    //     // std::cout << "GO!\n";
    // }
}

void Pr100Drive::showSensorData(void)
{
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
    int count = 0; int lock = 0;
    while (ros::ok())
    {
        // if (lock < 3)
        // {
        //     pr100_drive->controlLoop();
        //     lock++;
        // }

        pr100_drive->controlLoop();
        

        count++;
        if (count >= 50)
        {
            // pr100_drive->showSensorData();
            count = 0;            
        }        

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
