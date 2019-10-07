// This file solves the Swerve Steer's ICR-based kinematic model and then publishes its state
// Subscribe: 'control_effort' topic; The control effort is needed to simulate plant dynamics
// Publish: "state"; update robot state (x_p,y_p,theta) 

#include "../include/swerve_steer_plant.h"


int main(int argc, char **argv)
{
  ROS_INFO("Starting simulation of third-order, swerve steer robot.");
  ros::init(argc, argv, "swerve_plant");
    
  ros::NodeHandle plant_node;

  // Declare a new message variable
  fbl_control::plant_msg  msg;

  msg.t = t_IC;

  ROS_INFO("Checkpoint 2");
  // Publish a plant.msg
  ros::Publisher chatter_pub = plant_node.advertise<fbl_control::plant_msg>("state", 1);

  // Subscribe to "control_effort" topic to get a controller_msg.msg
  ros::Subscriber sub1 = plant_node.subscribe("control_effort", 1, chatterCallback );

  // Subscribe to "setpt" topic to get a setpt_msg.msg
  ros::Subscriber sub2 = plant_node.subscribe("setpt", 1, setptCallback );
  
  // Initial conditions -- these were defined in the header file
  for (int i=0; i<num_states; i++)
  {
    ROS_INFO("Setting initial state Conditions");
    msg.x[i] = x_IC[i];
  }
  for (int i=0; i<6; i++) {
    ROS_INFO("Setting initial setpt Conditions");
    msg.setpoint[i] = setpt[i];
  }

  ROS_INFO("Checkpoint 3");

  double x_dot [num_states] = {0};

  ros::Rate loop_rate(1/delta_t); // Control rate in Hz

  ROS_INFO("Checkpoint 4");

  while (ros::ok())
  {

    //Update setpts
    for (int i=0; i<6; i++) {
      ROS_INFO("Updating setpts");
      msg.setpoint[i] = setpt[i];
    }
    ROS_INFO("x1: %f   setpt: %f", msg.x[0], msg.setpoint[0]);
    ROS_INFO("x2: %f   setpt: %f", msg.x[1], msg.setpoint[1]);
    ROS_INFO("x3: %f   setpt: %f", msg.x[2], msg.setpoint[2]);

    ROS_INFO("u1: %f", u[0]);
    ROS_INFO("u2: %f", u[1]);
    ROS_INFO("u3: %f", u[2]);

    chatter_pub.publish(msg);
    
    x_dot[0] = -(W/2)*sin(msg.x[2])*u[2]+(L/2)*cos(msg.x[2])*u[2]+u[0];
    x_dot[1] = (W/2)*cos(msg.x[2])*u[2]+(L/2)*sin(msg.x[2])*u[2]+u[1];
    x_dot[2] = u[2];

    msg.x[0] = msg.x[0]+x_dot[0]*delta_t;
    msg.x[1] = msg.x[1]+x_dot[1]*delta_t;
    msg.x[2] = msg.x[2]+x_dot[2]*delta_t;

    msg.t = msg.t+delta_t;

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

// Callback 'control_effort' topic
void chatterCallback(const fbl_control::controller_msg& u_msg)
{
  //ROS_INFO("I heard: [%f]", u_msg.u[0]);

  // Define the stabilizing control effort
  u[0] = u_msg.u[0];
  u[1] = u_msg.u[1];
  u[2] = u_msg.u[2];
}

// Callback for 'setpt' topic
void setptCallback(const fbl_control::setpt_msg& setpt_msg)
{
  setpt[0] = setpt_msg.setx[0];
  setpt[1] = setpt_msg.setx[1];
  setpt[2] = setpt_msg.setx[2];
  setpt[3] = setpt_msg.setx[3];
  setpt[4] = setpt_msg.setx[4];
  setpt[5] = setpt_msg.setx[5];
}