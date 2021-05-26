
#include <ros/ros.h>

#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <time.h>
#include <unistd.h>

// constants
#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)
#define CLOCKWISE           -1
#define COUNTER_CLOCKWISE    1

// state machine
#define STATE_NORMAL   0
#define STATE_NARROW   1
#define STATE_TURN     2
#define STATE_ESCAPE   3

// speed
#define LINEAR_VELOCITY  0.1
#define ANGULAR_VELOCITY 0.1

// thresholds
#define THRESH_DIST          0.20 //0.32 //0.42 //0.5
#define THRESH_DIST_TIGHT    0.07
#define THRESH_POSE          0.005
#define THRESHOLD_CHECK_POSE 1000


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

