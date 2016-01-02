#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

/**
 * Escape behavior makes the robot backoff and turn when too close to obstacle
 */
class EscapeBehavior
{
public:
	// members

	// methods
  EscapeBehavior();

protected:
	// members
	ros::NodeHandle nh_,ph_;
    ros::Subscriber scan_sub_;
    ros::Publisher vel_pub_;
	ros::Timer timer_;

	int rate_; // update and publish rate (Hz)

    enum State
    {
      IDLE,
      BACKOFF,
      TURN,
    };
    State state_; // state of escape procedure
	ros::Time until_; // time until next state

	bool scan_received_; // at least one scan has been received
	double closest_distance_; // distance to closest object (m)
	double robot_size_; // size or diameter of robot (m)
	double bump_distance_; // robot size plus safety margin (m)
    double backoff_dur_; // duration of backoff (s)
	double backoff_vel_; // linear backoff velocity (m/s)
	double turn_dur_; // duration of turn (s)
	double turn_vel_; // angular turn velocity (rad/s)

	// methods
	void publish(double angular, double linear);
	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
	bool isTriggered(void);
	void update(void);
};

///////////////////////////////////////////////////////////////////////////

EscapeBehavior::EscapeBehavior():
  ph_("~"),
  rate_(50),
  state_(IDLE),
  scan_received_(false),
  robot_size_(0.33),
  bump_distance_(1.0),
  backoff_dur_(1.0),
  backoff_vel_(0.5),
  turn_dur_(1.0),
  turn_vel_(1.5)
{
  ph_.param("publish_rate", rate_, rate_);
  ph_.param("backoff_duration", backoff_dur_, backoff_dur_);
  ph_.param("backoff_velocity", backoff_vel_, backoff_vel_);
  ph_.param("turn_duration", turn_dur_, turn_dur_);
  ph_.param("turn_velocity", turn_vel_, turn_vel_);
  ph_.param("bump_distance", bump_distance_, bump_distance_);
  ph_.param("robot_size", robot_size_, robot_size_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 100, &EscapeBehavior::scanCallback, this);
  timer_ = nh_.createTimer(ros::Duration(1.0/rate_), boost::bind(&EscapeBehavior::update, this));
}

void EscapeBehavior::publish(double angular, double linear)  
{
  geometry_msgs::Twist vel;
  vel.angular.z = angular;
  vel.linear.x = linear;
  vel_pub_.publish(vel);    
  return;
}

void EscapeBehavior::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  closest_distance_ = scan_msg->range_max; 
  for (int i = 0; i < (int) scan_msg->ranges.size(); i++) 
  {
    double range = scan_msg->ranges[i];
    if ((range > scan_msg->range_min) && (range > robot_size_) && (range < closest_distance_)) 
    { // closest distance to obstacle, excluding parts of the robot
      closest_distance_ = range;
	  ROS_DEBUG("closest_distance: %f", closest_distance_);
    }
  }
  scan_received_ = true; // we have received a scan
}

bool EscapeBehavior::isTriggered(void) {
  // check if behavior is triggered
  return scan_received_ && (closest_distance_ < bump_distance_);
}

void EscapeBehavior::update(void)
{
  geometry_msgs::Twist twist_msg;
  ros::Time now = ros::Time::now();
  switch (state_)
  {
    case IDLE:
      if (isTriggered())
      {
        // start escape
        ROS_DEBUG("Escape triggered");
        state_ = BACKOFF;
        until_ = now + ros::Duration(backoff_dur_);
        ROS_DEBUG("Escape::BACKOFF");
      } else {
        // just be idle, i.e. do nothing
      }
      break;
    case BACKOFF:
      if(now < until_)
      {
        // backoff
		publish(0.0,-1*backoff_vel_);
      } else {
        state_ = TURN;
        until_ = ros::Time::now() + ros::Duration(turn_dur_);
        ROS_DEBUG("Escape::TURN");
      }
      break;
    case TURN:
      if(now < until_)
      {
        // turn
		publish(turn_vel_,0.0);
      } else {
        state_ = IDLE;
      }        
      break;
    default:
      // should not get here
      break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "escape_behavior");
  EscapeBehavior escape_behavior;
  ros::spin();

  return 0;
}
