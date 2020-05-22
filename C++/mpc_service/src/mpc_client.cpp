/*================================================================
*   Copyright (C) 2020 Sangfor Ltd. All rights reserved.
*   
*   @file:mpc_client.cpp
*   @author: Sophistt
*   @date:2020-05-18 14:54
*   @description: 
*
================================================================*/


#include "ros/ros.h"
#include "snopt_msgs/SnoptCal.h"
#include "cstdio"


int main(int argc, char **argv) 
{   
    ros::init(argc, argv, "mpc_client");

    if (argc != 3)
    {
        ROS_INFO("Usage: mpc_client constraints_1 constraints_2");
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<snopt_msgs::SnoptCal>("mpc");
    
    snopt_msgs::SnoptCal srv;
    srv.request.c1 = atoll(argv[1]);
    srv.request.c2 = atoll(argv[2]);

    if (client.call(srv))
    {
        ROS_INFO("X[0]: %ld, X[1]: %ld", (long int)srv.response.x[0], (long int)srv.response.x[1]);
    }
    else
    {
        ROS_ERROR("Failed");
        return 1;
    }

    return 0;
}

