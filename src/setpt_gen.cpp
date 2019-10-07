#include "../include/setpt_gen.h"

int main(int argc, char **argv)
{
	ROS_INFO("Starting setpt generator");
	ros::init(argc, argv, "setpt_gen");

	ros::NodeHandle setpt_node;

	// Declare a new message variable
	fbl_control::setpt_msg  msg;

	// Publish a plant.msg
  	ros::Publisher setpt_pub = setpt_node.advertise<fbl_control::setpt_msg>("setpt", 1);

	double t = 0.0;
	double delta_t = 0.01;

	ros::Rate loop_rate(1/delta_t); // Control rate in Hz

	while (ros::ok())
  	{
  		// Circle Trajectory
  		double x_d = 1 * cos(t);
  		double y_d = 1 * sin(t);
  		double theta_d = 0;

  		double vx_d = -1 * sin(t);
  		double vy_d = 1 * cos(t);
  		double w_d = 0;

  		msg.setx = {x_d, y_d, theta_d, vx_d, vy_d, w_d};

	    setpt_pub.publish(msg);

	    t = t+delta_t;

	    ros::spinOnce();
	    loop_rate.sleep();
  	}

  return 0;
}

