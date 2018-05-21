#include <vector>
#include "mrobot_bringup/mrobot.h"

namespace mrobot
{

const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char header[2] = {0x55, 0xaa};
const int SPEED_INFO = 0xa55a; 
const int GET_SPEED  = 0xaaaa;
const double ROBOT_RADIUS = 105.00;
const double ROBOT_LENGTH = 210.50;

boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyUSB0");

boost::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};
boost::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0, 
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};

union sendData
{
	int d;
	unsigned char data[4];
}leftdata, rightdata;

union checkSum
{
	short d;
	unsigned char data[1];
}SendCheckSum, ReceiveCheckSum;

union receiveHeader
{
	int d;
	unsigned char data[2];
}receive_command, receive_header;

union sendCommand
{
	int d;
	unsigned char data[2];
}send_command;

union odometry
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}vel_left, vel_right;

MRobot::MRobot():
    x_(0.0), y_(0.0), th_(0.0),
    vx_(0.0), vy_(0.0), vth_(0.0)
{
}

MRobot::~MRobot()
{
}

bool MRobot::init()
{
    // 串口连接
	sp.set_option(boost::asio::serial_port::baud_rate(115200));
	sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
	sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	sp.set_option(boost::asio::serial_port::character_size(8));
    
    ros::Time::init();
	current_time_ = ros::Time::now();
	last_time_ = ros::Time::now();
	
    //定义发布消息的名称
    pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);		
    
    return true;
}

bool MRobot::readSpeed()
{
	int i, length = 0;
	unsigned char checkSum;
    unsigned char buf[200];
    
    // 读取串口数据
	boost::asio::read(sp, boost::asio::buffer(buf));
	ros::Time curr_time;
	for (i = 0; i < 2; i++)
		receive_header.data[i] = buf[i];
	
    // 检查信息头
	if (receive_header.data[0] != header[0] || receive_header.data[1] != header[1])
	{
		ROS_ERROR("Received message header error!");
        return false;
	}

	for (i = 0; i < 2; i++)
		receive_command.data[i] = buf[i + 2];
	
	length = buf[4];
	checkSum = getCrc8(buf, 5 + length);
    
    // 检查信息类型
    if(receive_command.d != SPEED_INFO)
	{
		ROS_ERROR("Received command error!");
        return false;
	}

    // 检查信息尾
    if (buf[6 + length] != ender[0] || buf[6 + length + 1] != ender[1])
	{
		ROS_ERROR("Received message header error!");
        return false;
	}
    
    // 检查信息校验值
    ReceiveCheckSum.data[0] = buf[5 + length];
    if (checkSum != ReceiveCheckSum.d)
	{
		ROS_ERROR("Received data check sum error!");
        return false;
	}

    // 读取速度值
    for(i = 0; i < 4; i++)
    {
        vel_left.odometry_char[i]  = buf[i + 5];
        vel_right.odometry_char[i] = buf[i + 9];
    }

    // 积分计算里程计信息
    vx_  = (vel_right.odoemtry_float + vel_left.odoemtry_float) / 2 / 1000;
    vth_ = (vel_right.odoemtry_float - vel_left.odoemtry_float) / ROBOT_LENGTH;
    
    curr_time = ros::Time::now();

    double dt = (curr_time - last_time_).toSec();
    double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
    double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
    double delta_th = vth_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;
    last_time_ = curr_time;               

	return true;
}

void MRobot::writeSpeed(double RobotV, double YawRate)
{
	unsigned char buf[16] = {0};
	int i, length = 0;
	double r = RobotV / YawRate;

    // 计算左右轮期望速度
	if(RobotV == 0)
	{
		leftdata.d = -YawRate * ROBOT_RADIUS;
		rightdata.d = YawRate * ROBOT_RADIUS;
	} 
    else if(YawRate == 0)
	{
		leftdata.d = RobotV;
		rightdata.d = RobotV;
	}
	else
	{
		leftdata.d  = YawRate * (r - ROBOT_RADIUS);
		rightdata.d = YawRate * (r + ROBOT_RADIUS);
	}

    // 设置消息头
	for(i = 0; i < 2; i++)
		buf[i] = header[i];
	
    // 设置消息类型
	send_command.d = GET_SPEED;
	for(i = 0; i < 2; i++)
		buf[i + 2] = send_command.data[i];
	
    // 设置机器人左右轮速度
	length = 8;
	buf[4] = length;
	for(i = 0; i < 4; i++)
	{
		buf[i + 5] = rightdata.data[i];
		buf[i + 9] = leftdata.data[i];
	}
    
    // 设置校验值、消息尾
	buf[5 + length] = getCrc8(buf, 5 + length);
	buf[6 + length] = ender[0];
	buf[6 + length + 1] = ender[1];

    // 通过串口下发数据
	boost::asio::write(sp, boost::asio::buffer(buf));
}

bool MRobot::spinOnce(double RobotV, double YawRate)
{
    // 下发机器人期望速度
    writeSpeed(RobotV, YawRate);

    // 读取机器人实际速度
    readSpeed();

    current_time_ = ros::Time::now();
    // 发布TF
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id  = "base_footprint";

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromYaw(th_);
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    odom_broadcaster_.sendTransform(odom_trans);

    // 发布里程计消息
    nav_msgs::Odometry msgl;
    msgl.header.stamp = current_time_;
    msgl.header.frame_id = "odom";

    msgl.pose.pose.position.x = x_;
    msgl.pose.pose.position.y = y_;
    msgl.pose.pose.position.z = 0.0;
    msgl.pose.pose.orientation = odom_quat;
    msgl.pose.covariance = odom_pose_covariance;

    msgl.child_frame_id = "base_footprint";
    msgl.twist.twist.linear.x = vx_;
    msgl.twist.twist.linear.y = vy_;
    msgl.twist.twist.angular.z = vth_;
    msgl.twist.covariance = odom_twist_covariance;
  
    pub_.publish(msgl);
}

unsigned char MRobot::getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}

}
