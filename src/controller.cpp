
// Subscribe to a topic about the state of a dynamic system and calculate feedback to
// stabilize it.
// Should run at a faster loop rate than the plant.


#include "../include/controller.h"
#include <iostream>
int main(int argc, char **argv)
{
    ROS_INFO("Check the num. of states and num. of inputs.");
    ROS_INFO("These can be adjusted in 'controller_msg.msg' and 'plant_msg.msg' files.");
    ROS_INFO("num_states: %i num_inputs: %i",num_states,num_inputs);
    ROS_INFO(" ");
    ROS_INFO("Also check the parameters of the controller at the top of fbl_control/include/controller.h");

    // ROS stuff
    ros::init(argc, argv, "controller");
    ros::NodeHandle controller_node;
    ros::Publisher chatter_pub = controller_node.advertise<fbl_control::controller_msg>("control_effort", 1);
    ros::Subscriber sub = controller_node.subscribe("state", 1, chatterCallback );

    // Open Loop Controller Tower Publisher
    ros::Publisher control_tower = controller_node.advertise<std_msgs::Float32MultiArray>("control_tower", 1);

    // Initialize motor_dat
    for (int i=0;i<8;i++) 
    {
        motor_dat.data.push_back(0.0);
    }
    // Main loop
    while (ros::ok())
    {
        ros::spinOnce();

        // Publish the stabilizing control effort
        control_tower.publish(motor_dat);
        chatter_pub.publish(u_msg);
    }

    return 0;
}


///////////////////////////////////////////////////////////////
// The main callback where a stabilizing u is calculated.
// This is where the magic happens.
///////////////////////////////////////////////////////////////
void chatterCallback(const fbl_control::plant_msg& msg)
{
    ROS_INFO("I heard x: %f %f", msg.x[0]);

    if ( first_callback )
    {
        initial_error_check(msg);
    }
    //Convert the message into the boost type we need
    for (int i=0; i<num_states; i++)
    {
        x(i)=msg.x[i];
    }
    for (int i=0; i<6; i++)
    {
        setpt(i) = msg.setpoint[i];
    }
    
    double k_p = 0.0;
    double r = 0.1;
    double drive_max = 10.0;
    Eigen::Matrix3d invG;
    Eigen::Vector3d del_x;
    Eigen::Vector3d max_del_x;
    Eigen::Vector3d u_tilda;
    Eigen::Vector3d vel_d;
	
    Eigen::Matrix3d G;
    double a =  -(W/2)*sin(x(2))+(L/2)*cos(x(2));
    double b =  (W/2)*cos(x(2))+(L/2)*sin(x(2));
    G << 1, 0, a,
 0, 1, b,
 0, 0, 1;
    invG = G.inverse();
    int zc = 0;
    // Calculate del_x
    for (int i=0; i<num_states; i++)
    {
        max_del_x(i) = abs(setpt(i)-x(i));
        del_x(i) = setpt(i)-x(i); 
        if(max_del_x(i) == 0) {
            zc = zc + 1;
        }
    }

    ROS_INFO("CALCULATED DEL_X");
    // Calcule k_d desired
    if(zc >= 3) {
        k_p = 0.1;
    } else {
        k_p = sqrt((r*drive_max)/((pow(0.5*max_del_x(2)+max_del_x(0),2)+pow(0.5*max_del_x(2)+max_del_x(1),2))));
    }
    
    std::cout<<"k_p"<<k_p;
    // Calculate u
    u_tilda = k_p*del_x;

    for(int i=0; i<3; i++) {
        vel_d(i) = setpt(i+3);
    }

    u = invG*(u_tilda+vel_d);
    ROS_INFO("FINISHED U CALC");
    /*Calculate G
    boost::numeric::ublas::matrix<double> G (num_inputs, num_states);
    G(0,0) = 1; G(0,1) = 0; G(0,2) = -(W/2)*sin(x(3))-(L/2)*cos(x(3));
    G(1,0) = 0; G(1,1) = 1; G(1,2) = (W/2)*cos(x(3))+(L/2)*sin(x(3));
    G(2,0) = 0; G(2,1) = 0; G(2,2) = 1;

    //Calculate del_x
    for (int i=0; i<num_states; i++)
	del_x[i] = abs(setpoint[i]-x[i]);

    // Calculate k_p desired
    k_p = sqrt((r*drive_max)/((pow(0.5*del_x[2]+del_x[0],2)+pow(0.5*del_x[2]+del_x[1],2)))); 

    // Calculate u
    u_tilda = k_p*del_x;
    u = prod(inverse(G),u_tilda);
    */
    //Check control effort saturation
    for (int i=0; i<num_inputs; i++)
    {
        if ( u(i) < low_saturation_limit[i] )
        {
            u(i) = low_saturation_limit[i];
        }
        if ( u(i) > high_saturation_limit[i] )
        {
            u(i) = high_saturation_limit[i];
        }
    }

    // del_t is time increment
    double xe2_dot = 0.0;
    double ye2_dot = 0.0;
    double xe3_dot = 0.0;
    double ye3_dot = 0.0;
    double xe4_dot = 0.0;
    double ye4_dot = 0.0;

    double x1_dot = (x(0) - prev_x[0])/del_t;
    double y1_dot = (x(1) - prev_x[1])/del_t;

    // SECOND WHEEL VELOCITIES
    xe2_dot = -(W/2)*u(2)*sin(x(2)) - (L/2)*u(2)*cos(x(2)) + x1_dot;
    ye2_dot = (W/2)*u(2)*cos(x(2)) - (L/2)*u(2)*sin(x(2)) + y1_dot;

    // THIRD WHEEL VELOCITIES
    xe3_dot = -(W/2)*u(2)*sin(x(2)) + (L/2)*u(2)*cos(x(2)) + x1_dot;
    ye3_dot = (W/2)*u(2)*cos(x(2)) + (L/2)*u(2)*sin(x(2)) + y1_dot;

    // FOURTH WHEEL VELOCITIES
    xe4_dot = (W/2)*u(2)*sin(x(2)) + (L/2)*u(2)*cos(x(2)) + x1_dot;
    ye4_dot = -(W/2)*u(2)*cos(x(2)) + (L/2)*u(2)*sin(x(2)) + y1_dot;

    // Assume no slip
    double wheel_radius = 0.1016; // (meters)
    double des_steer[4];
    des_steer[0] = atan2(u(1),u(0))-x(2)-(3.14159265/2.0);
    double drive_speed_e1 = (sqrt( pow(u(0),2) + pow(u(1),2) )) / wheel_radius;

    des_steer[1] = atan2(ye2_dot,xe2_dot)-x(2)-(3.14159265/2.0);
    double drive_speed_e2 = (sqrt( pow(xe2_dot,2) + pow(ye2_dot,2) )) / wheel_radius;

    des_steer[2] = atan2(ye3_dot,xe3_dot)-x(2)-(3.14159265/2.0);
    double drive_speed_e3 = (sqrt( pow(xe3_dot,2) + pow(ye3_dot,2) )) / wheel_radius;

    des_steer[3] = atan2(ye4_dot,xe4_dot)-x(2)-(3.14159265/2.0);
    double drive_speed_e4 = (sqrt( pow(xe4_dot,2) + pow(ye4_dot,2) )) / wheel_radius;

    // Fix negatives
    for (int i=0; i<4; i++) 
    {
        if (des_steer[i] < 0) 
        {
            des_steer[i] = des_steer[i] + 2 * 3.14159265;
        }
    }

    // PUBLISH desired steer and drive velocities to /controller_commands
    std_msgs::Float32MultiArray array;
    //Clear array
    array.data.clear();
    
    float pts[8] = {des_steer[0], des_steer[1], des_steer[2], des_steer[3], drive_speed_e1, drive_speed_e2, drive_speed_e3, drive_speed_e4};
    std::vector<float> data_vec;
    //ROS_INFO("MOTOR data: ");
    for (int i=0; i<8; i++) {
        data_vec.push_back(pts[i]);
        array.data.push_back(pts[i]);
        std::cout << pts[i] << " ";
    }
    motor_dat = array;

    // Stuff the message to be published
    for (int i=0; i<num_inputs; i++)
        u_msg.u[i] = u(i);

    // Change prev x
    for (int i=0; i < num_states; i++) {
        prev_x[i] = x(i);
    }
    
    //ROS_INFO("Calculated u: %f", u_msg.u[0]);
}

////////////////////////////////////////////////////////////////////
// Initial error check
////////////////////////////////////////////////////////////////////
void initial_error_check(const fbl_control::plant_msg& msg)
{
    if ( (sizeof(high_saturation_limit)+sizeof(low_saturation_limit)) / sizeof(*high_saturation_limit) != 2*num_inputs )
    {
        ROS_ERROR("Check your saturation limit definitions. There are too many or too few values in one of the arrays. They must match the # of inputs to the system.");
        ros::shutdown();
    }

    for (int i=0; i<num_inputs; i++)
        if ( low_saturation_limit[i] >= high_saturation_limit[i] )
        {
            ROS_ERROR("The 'low saturation limit' is higher than 'high saturation limit.' Change them in controller_parameters.h.");
            ros::shutdown();
        };

    if ( msg.setpoint.size() != num_states )
    {
        ROS_ERROR("The published setpoint's length does not equal # of states. Check the data which is published by the plant.");
        ros::shutdown();
    }
}
