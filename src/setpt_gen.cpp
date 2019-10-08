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
  		double t_i = 0.0;
  		double t_end = 6.2831853;

  		double t_1 = 0.0;
  		double t_2 = 6.2831853;
  		double S = 0;
  		int c = 0;

  		if (t_i <= t && t < t_1) {
  			double m = 1/(t_1-t_i);
  			S = m*t;
  		} else if (t_1 <= t && t < t_2) {
  			S = 1;
  		} else if (t_2 <= t && t < t_end) {
  			double m = -1/(t_end-t_2);
  			S = m*t;
  		} else if (t >= t_end){
  			ROS_INFO("YESS");
  			S = 0;
  			c = 1;
  		}

  		ROS_INFO("t, c %f, %f", t, c);
  		if(c == 1) {
  			double x_d = 1 * cos(t_end/2);
  			double y_d = 1 * sin(t_end/2);
  			double theta_d = 0;

  			double vx_d = S*(-sin(t_end/2));
  			double vy_d = S*(cos(t_end/2));
  			double w_d = 0;
  			msg.setx = {x_d, y_d, theta_d, vx_d, vy_d, w_d};
  		} else {
  			double x_d = 1 * cos(t/2);
  			double y_d = 1 * sin(t/2);
	  		double theta_d = 0;

	  		double vx_d = S*(-sin(t/2));
	  		double vy_d = S*(cos(t/2));
	  		double w_d = 0;
	  		msg.setx = {x_d, y_d, theta_d, vx_d, vy_d, w_d};
  		}

  		/*Y=0.5 Trajectory
  		double x_d = 0.5;	
  		double y_d = 0;
  		double theta_d = 0;

  		double vx_d = 0;
  		double vy_d = 0;
  		double w_d = 0;

  		msg.setx = {x_d, y_d, theta_d, vx_d, vy_d, w_d};*/

	    setpt_pub.publish(msg);

	    t = t+delta_t;

	    ros::spinOnce();
	    loop_rate.sleep();
  	}

  return 0;
}

