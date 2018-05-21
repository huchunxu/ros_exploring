#ifndef MROBOT_H
#define MROBOT_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>

namespace mrobot
{

class MRobot
{
public:
    MRobot();
    ~MRobot();
    bool init();
    bool spinOnce(double RobotV, double YawRate);
   
private:
    bool readSpeed();
    void writeSpeed(double RobotV, double YawRate);
    unsigned char getCrc8(unsigned char *ptr, unsigned short len);
   
private:
    ros::Time current_time_, last_time_;

    double x_;
    double y_;
    double th_;

    double vx_;
    double vy_;
    double vth_;

    ros::NodeHandle nh;
    ros::Publisher pub_;
    tf::TransformBroadcaster odom_broadcaster_;
};
    
}

#endif /* MROBOT_H */
