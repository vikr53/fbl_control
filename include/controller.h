#pragma once
#include <math.h>
// Using ublas for vector/matrix multiplication
#include <bits/stdc++.h> 

#include "std_msgs/Float32MultiArray.h"
#include "ros/ros.h"
#include "fbl_control/plant_msg.h"
#include "fbl_control/controller_msg.h"

#include "./controller_globals.h"


///////////////////////////////////////////////////////////////////////////////
// User-defined parameters - MAKE YOUR CHANGES HERE
///////////////////////////////////////////////////////////////////////////////

static const double high_saturation_limit [] = {10, 10, 10};
static const double low_saturation_limit []= {-10, -10, -10};

// The state space definition-- the dynamic equations of the model.
// Calculates dx/dt
// model_definition sees u b/c it's a global variable, it can't be an argument
/*void model_definition(const ublas_vector &x, ublas_vector &dxdt, const double t)
{
  dxdt[0] = -(W/2)*sin(x[2])*u[2]+(L/2)*cos(x[2])*u[2]+u[0];
  dxdt[1] = (W/2)*cos(x[2])*u[2]+(L/2)*sin(x[2])*u[2]+u[1];
  dxdt[2] = u[2];
}*/

/////////////////////////////////////////////////////////////////////////////
// Functions
/////////////////////////////////////////////////////////////////////////////

// The main callback to calculate u
void chatterCallback(const fbl_control::plant_msg& msg);

void initial_error_check(const fbl_control::plant_msg& msg);
