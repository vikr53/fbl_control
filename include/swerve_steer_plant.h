#include <math.h>
#include "ros/ros.h"

// Header for 'plant_msg.msg'
#include "fbl_control/plant_msg.h"

// Header for controller_msg.msg
#include "fbl_control/controller_msg.h"

// Header for setpt_msg.msg
#include "fbl_control/setpt_msg.h"

/////////////////////////////////////////////
// Variables -- Make changes here.
/////////////////////////////////////////////

static const int num_states=3;
static const int num_inputs=3;

// Robot dimensions: 17 inches
static const double W = 0.4318;
static const double L = 0.4318;

// Initial conditions
static const double x_IC [num_states] = {0,0,0};
static const double t_IC = 0.0;
std::vector<double> setpoint (3);

double delta_t = 0.01; // control period in seconds

// Global so it can be passed from the callback fxn to main
static double u[num_inputs] = {0};
static double setpt[6] = {0};


/////////////////////////////////////////////
// Functions
/////////////////////////////////////////////

void chatterCallback(const fbl_control::controller_msg& u_msg);
void setptCallback(const fbl_control::setpt_msg& setpt_msg);
