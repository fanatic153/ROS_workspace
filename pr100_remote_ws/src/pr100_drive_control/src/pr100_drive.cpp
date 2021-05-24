
#include <ros/ros.h>

#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <time.h>
#include <unistd.h>

// constants
#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)
#define CLOCKWISE           1
#define COUNTER_CLOCKWISE  -1

// state machine
#define STATE_NORMAL   0
#define STATE_NARROW   1
#define STATE_TURN      2
#define STATE_ESCAPE   3

// speed
#define LINEAR_VELOCITY  0.1
#define ANGULAR_VELOCITY 0.1

// thresholds
#define THRESH_DIST 0.20 //0.32 //0.42 //0.5
#define THRESH_DIST_TIGHT 0.07
#define THRESH_POSE 0.005
#define THRESHOLD_CHECK_POSE 2000

class Pr100Drive
{
 public:

    Pr100Drive();
    ~Pr100Drive();

    int currentState = STATE_NORMAL;    

    void init(void);
    void controlLoop(void);
    void showSensorData(void);
    void showState(void);
    void updatePose(void);

    // state mode
    void stateNormal(void);
    void stateNarrow(void);
    void stateTurn(void);
    void stateEscape(void);

 private:

    // Variables
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Subscriber sonar1_sub;
    ros::Subscriber sonar2_sub;
    ros::Publisher cmd_vel_pub;

    double pose;
    double roboX, roboY, roboX_prev, roboY_prev;
    double range1, range2;
    
    int update_time = 0;
    time_t t_mark;

    void sonar1MsgCallBack(const sensor_msgs::Range::ConstPtr &msg);
    void sonar2MsgCallBack(const sensor_msgs::Range::ConstPtr &msg);
    void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
    void updatecommandVelocity(double linear, double angular);

    void turnTheta(double theta, int direction);
    void goForward(void);
    void goBackward(void);
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

    roboX = msg->pose.pose.position.x;
    roboY = msg->pose.pose.position.y;


    if (update_time <= 0)
    {
        std::cout << "update: pose_prev \n";
        updatePose();

        update_time = THRESHOLD_CHECK_POSE;
    }     
    update_time--; 

    // std::cout << "pose = " << pose << "\n";        
}

void Pr100Drive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub.publish(cmd_vel);
}

void Pr100Drive::turnTheta(double theta, int direction)
{
    // double t_turn = (theta * DEG2RAD) / ANGULAR_VELOCITY;
    unsigned int t_turn = 100000*(theta * DEG2RAD) / ANGULAR_VELOCITY;
    // std::cout << "t_turn = " << t_turn << "\n";

    t_mark = time(NULL);

    updatecommandVelocity(0.0, direction * ANGULAR_VELOCITY);
    usleep(t_turn);    // sleep in micro-sec
    updatecommandVelocity(0.0, 0.0);

    // std::cout << "time elapsed: " << time(NULL) - t_mark << "\n";
    
}

void Pr100Drive::goForward(void)
{
    updatecommandVelocity(LINEAR_VELOCITY , 0.0);
}

void Pr100Drive::goBackward(void)
{
    updatecommandVelocity((-1) * LINEAR_VELOCITY , 0.0);
}

void Pr100Drive::stop(void)
{
    updatecommandVelocity(0.0 , 0.0);  
}

void Pr100Drive::stateNormal(void)
{
    goForward();
}

void Pr100Drive::stateNarrow(void)
{
    goForward();
}

void Pr100Drive::stateTurn(void)
{
    double theta = 30;
    if (range1 > range2)
    {
        turnTheta(theta, CLOCKWISE);
    }
    else
    {
        turnTheta(theta, COUNTER_CLOCKWISE);             
    }
}

void Pr100Drive::stateEscape(void)
{
    goBackward();
    usleep(1000000);    // sleep in micro-sec

    double theta = 180;
    turnTheta(theta, CLOCKWISE);
    usleep(1000000);    // sleep in micro-sec
}

void Pr100Drive::controlLoop(void)
{
    double theta = 30;

    currentState = STATE_NORMAL;

    // going into an obstacle
    if (range1 < THRESH_DIST && range2 < THRESH_DIST)
    {
        currentState = STATE_NARROW;

        // straight hitting into an obstacle
        if (range1 < THRESH_DIST_TIGHT && range2 < THRESH_DIST_TIGHT)
        {
            currentState = STATE_ESCAPE;
        }
    }    
    // getting close to an obstale
    else if (range1 < THRESH_DIST || range2 < THRESH_DIST)
    {
        currentState = STATE_TURN;
    }

    // stucked
    if ((roboX - roboX_prev) * (roboX - roboX_prev) < THRESH_POSE * THRESH_POSE && 
        (roboY - roboY_prev) * (roboY - roboY_prev) < THRESH_POSE * THRESH_POSE)
    {
        currentState = STATE_ESCAPE;
    }
}

// void Pr100Drive::controlLoop(void)
// {
//     double theta = 30;

//     if (range1 < 0.2 && range2 < 0.2)
//     {
//         goBackward();
//     }
//     else if (range1 < DISTANCE_THRESH || range2 < DISTANCE_THRESH)
//     {
//         if (range1 > range2)
//         {
//             turnTheta(theta, CLOCKWISE);
//         }
//         else
//         {
//             turnTheta(theta, COUNTER_CLOCKWISE);             
//         }
        
//         // updatecommandVelocity(0.0 , ANGULAR_VELOCITY);
//         // std::cout << "Turn!\n";
//     }
//     else
//     {
//         updatecommandVelocity(LINEAR_VELOCITY , 0.0);    
//         // std::cout << "GO!\n";
//     }
// }

void Pr100Drive::showState(void)
{    
    switch (currentState)
    {
        case STATE_NORMAL:
            std::cout << "STATE_NORMAL!\n";            
            break;
        case STATE_TURN:            
            std::cout << "STATE_TURN!\n"; 
            break;
        case STATE_NARROW:
            std::cout << "STATE_NARROW!\n"; 
            break;
        case STATE_ESCAPE:
            std::cout << "STATE_ESCAPE!\n"; 
            break;
        default: 
            break;
    }
}

void Pr100Drive::updatePose(void)
{
    roboX_prev = roboX;
    roboY_prev = roboY;
}

void Pr100Drive::showSensorData(void)
{
    std::cout << "sonar1, sonar2 = \n";
    std::cout << "  " << range1 << "\n";
    std::cout << "  " << range2 << "\n";

    std::cout << "pose, prev = \n"; 
    std::cout << "  " << pose << "\n";
    // std::cout << "  " << prev_tb3_pose_ << "\n";    
    std::cout << "x, y = " << roboX << ", " << roboY << "\n";  
    std::cout << "     = " << roboX_prev << ", " << roboY_prev << "\n";      

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
    
    int prevState = pr100_drive->currentState;
    pr100_drive->updatePose();
    while (ros::ok())
    {        
        count++;
        if (count >= 50)
        {
            pr100_drive->showSensorData();
            pr100_drive->showState();

            count = 0;
        }

        // state control
        pr100_drive->controlLoop();
        // state machine
        switch (pr100_drive->currentState)
        {
            case STATE_NORMAL:
                pr100_drive->stateNormal();
                break;

            case STATE_TURN:
                pr100_drive->stateTurn();
                break;

            case STATE_NARROW:
                pr100_drive->stateNarrow();
                break;

            case STATE_ESCAPE:
                pr100_drive->stateEscape();
                break;

            default:
                break;
        }

        // show switched state
        if (prevState != pr100_drive->currentState)
        {
            pr100_drive->showState();
        }
        // saving state
        prevState = pr100_drive->currentState;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
