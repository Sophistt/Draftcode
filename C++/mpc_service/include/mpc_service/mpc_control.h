/*================================================================
*   Copyright (C) 2020 Sangfor Ltd. All rights reserved.
*   
*   @file:mpc_control.h
*   @author: Sophistt
*   @date:2020-05-18 11:07
*   @description: 
*
================================================================*/


#ifndef MPC_CONTROL_H
#define MPC_CONTROL_H

#include <iostream>

#include <ros/ros.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "snopt_msgs/SnoptCal.h"
#include "autopilot_control_msgs/PathTrackByMpc.h"
#include "snoptProblem.hpp"

struct VehicleState
{
    double x;      // x-postion
    double y;      // y-postion -- lateral offset 
    double v;      // velocity 
    double theta;  // yaw angle 
};


class MpcControl
{

private:
    
    // Vehicle Vairables
    const double kWheelBase = 2.7;
    
    // Mpc Variables
    const int kNumsOfStateVector = 4, kHorizonLength = 5;
    const double kTimeTick = 0.1; // s

    Eigen::MatrixXd oper_path_;    // For calculate linear model matrix

    Eigen::MatrixXd state_mat_;    // TODO Maybe need to use static variable 
    Eigen::Matrix4d state_transit_mat_;  // Matrix A (4 x 4)
    Eigen::Vector4d control_transit_mat_, const_transit_mat_;  // Matrix B (4, 1), Matrix C (4, 1)

    void CalLinearModelMat(double vel, double theta, double delta);
    
    void PredictMotion(const VehicleState *vehicle_state_ptr,
                       autopilot_control_msgs::PathTrackByMpc::Request *req);

    void UpdateVehicleState(VehicleState &state, double delta);

    void AddTransitConstraints(double x, double xx, 
                               double y, double yy,
                               double theta, double theta_t,
                               double v, double &delta);
    

public:
    MpcControl();
    ~MpcControl();

    double* Predict();
    double* Predict(VehicleState *vehicle_state, autopilot_control_msgs::PathTrackByMpc::Request *req);

    int GetHorizontLength();
};


inline int MpcControl::GetHorizontLength()
{
    return kHorizonLength;
}


#endif
