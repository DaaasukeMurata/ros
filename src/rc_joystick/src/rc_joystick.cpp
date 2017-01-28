#include <ros/ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt16MultiArray.h>
#include <sensor_msgs/Joy.h>

class TeleopRc
{
  public:
	TeleopRc();

  private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	ros::NodeHandle nh;
	int vel_linear, vel_angular;
	double l_scale_, a_scale_;
	ros::Publisher  vel_pub_;
	ros::Subscriber joy_sub_;
};

TeleopRc::TeleopRc(): vel_linear(3), vel_angular(0), a_scale_(90.0),l_scale_(45.0)
{
	nh.param("axis_linear"  , vel_linear , vel_linear);
	nh.param("axis_angular" , vel_angular, vel_angular);
	nh.param("scale_angular", a_scale_, a_scale_);
	nh.param("scale_linear" , l_scale_, l_scale_);

	joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopRc::joyCallback, this);
	vel_pub_ = nh.advertise<std_msgs::UInt16MultiArray>("servo", 1);
}

void TeleopRc::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	std_msgs::UInt16MultiArray cmd_msg;
	cmd_msg.data.clear();
	cmd_msg.data.push_back((uint16_t)((-1.0)*a_scale_*joy->axes[vel_angular] + 90.0));
	cmd_msg.data.push_back((uint16_t)((-1.0)*l_scale_*joy->axes[vel_linear] + 90.0));
	ROS_INFO_STREAM("(" << joy->axes[vel_linear] << " " << joy->axes[vel_angular] << ")");
	vel_pub_.publish(cmd_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_rc");
	TeleopRc teleop_rc;

	ros::spin();
	return 0;
}

