#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

/**
 * Cruise behavior makes the robot cruise, i.e. drive straight forward.
 */
class CruiseBehavior
{
public:
	// members

	// methods
  CruiseBehavior();

protected:
	// members
	ros::NodeHandle nh_,ph_;
    ros::Publisher vel_pub_;
	ros::Timer timer_;

	int rate_; // update and publish rate (Hz)
	double cruise_vel_; // cruise velocity (m/s)

	// methods
	void publish(double angular, double linear);
	bool isTriggered(void);
	void update(void);
};

///////////////////////////////////////////////////////////////////////////

CruiseBehavior::CruiseBehavior():
  ph_("~"),
  rate_(10),
  cruise_vel_(0.5)
{
  ph_.param("publish_rate", rate_, rate_);
  ph_.param("cruise_velocity", cruise_vel_, cruise_vel_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  timer_ = nh_.createTimer(ros::Duration(1.0/rate_), boost::bind(&CruiseBehavior::update, this));
}

void CruiseBehavior::publish(double angular, double linear)  
{
  geometry_msgs::Twist vel;
  vel.angular.z = angular;
  vel.linear.x = linear;
  vel_pub_.publish(vel);    
  return;
}

bool CruiseBehavior::isTriggered(void) {
  return true; // always triggered
}

void CruiseBehavior::update(void)
{
  if (isTriggered())
  {
  	publish(0.0,cruise_vel_);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "escape_behavior");
  CruiseBehavior cruise_behavior;
  ros::spin();

  return 0;
}
