
#pragma once

//Eigen++
#include "Eigen/Dense"

/////////////////////////////////////////////////////////////////////////////
// These variables are global b/c a ROS callback only takes 1 argument
/////////////////////////////////////////////////////////////////////////////

static double t=0; // time will be updated by listening to the 'plant' ROS topic
static int first_callback=1; // 1 signals that the callback has not been run yet. Triggers setup calcs

// Robot dimensions
static const double W = 0.4318;
static const double L = 0.4318;
static const double del_t = 0.01;

// Read the size of a plant_msg
fbl_control::plant_msg temp_plant_msg; // Just to read the msg size
const static int num_states = temp_plant_msg.x.size();

// Message variable for the control effort message
fbl_control::controller_msg  u_msg;
// Read the size of a 'controller' message
const static int num_inputs = u_msg.u.size();

// Previous x vector global - hardcoded RN
double prev_x [3] = { };

Eigen::Vector3d x;
Eigen::Vector3d setpoint;
Eigen::Vector3d u;

std_msgs::Float32MultiArray motor_dat;
