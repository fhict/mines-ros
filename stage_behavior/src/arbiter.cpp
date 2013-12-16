#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

/**
 * Priority based arbiter, forwards highest priority commands
 */
class Arbiter
{
public:
	// members

	// methods
  Arbiter();

protected:
	// members

	// methods
};

///////////////////////////////////////////////////////////////////////////

Arbiter::Arbiter()
{

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "arbiter");
  Arbiter arbiter;
  ros::spin();

  return 0;
}
