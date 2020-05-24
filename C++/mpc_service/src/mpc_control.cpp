/*================================================================
*   Copyright (C) 2020 Sangfor Ltd. All rights reserved.
*   
*   @file:mpc_control.cpp
*   @author: Sophistt
*   @date:2020-05-18 11:06
*   @description: 
*
================================================================*/


#include "mpc_control.h"



MpcControl::MpcControl() 
{
    std::cout << "MPC Control Init." << std::endl;

    // Mpc Variables Init
    oper_path_ = Eigen::MatrixXd::Zero(kNumsOfStateVector, kHorizonLength + 1);
    
    state_mat_ = Eigen::MatrixXd::Zero(kNumsOfStateVector, kHorizonLength + 1);
    state_transit_mat_ = Eigen::Matrix4d::Zero();
    control_transit_mat_ = Eigen::Vector4d::Zero();
    const_transit_mat_ = Eigen::Vector4d::Zero();
}


MpcControl::~MpcControl()
{   
    std::cout << "MPC Cotrol deconstruct." << std::endl;
}


// Update vehicle state according to current speed and steering angle by kinematic model.
void MpcControl::UpdateVehicleState(VehicleState &state, double delta)
{
    state.x = state.x + state.v * std::cos(state.theta) * kTimeTick;
    state.y = state.y + state.v * std::sin(state.theta) * kTimeTick;
    state.theta = state.theta + state.v / kWheelBase * std::tan(delta) * kTimeTick;
}


// Predict the next horizonal-time vehicle states to calculate transition matrix.
void MpcControl::PredictMotion(const VehicleState *vehicle_state_ptr,
        autopilot_control_msgs::PathTrackByMpc::Request *req)
{  
    oper_path_.setZero();

    VehicleState vehicle_state = {vehicle_state_ptr->x, vehicle_state_ptr->y, 
                                   vehicle_state_ptr->v, vehicle_state_ptr->theta};

    for (int i = 0; i < kHorizonLength + 1; i++) {
        // if (i > 0) UpdateVehicleState(vehicle_state, x_[i]);
        oper_path_(0, i) = vehicle_state.x;
        oper_path_(1, i) = vehicle_state.y;
        oper_path_(2, i) = vehicle_state.v;
        oper_path_(3, i) = vehicle_state.theta;
    }
}


void MpcControl::CalLinearModelMat(double vel, double theta, double delta)
{   
    state_transit_mat_.setZero();
    control_transit_mat_.setZero();
    const_transit_mat_.setZero();
        
    // Matrix A
    state_transit_mat_(0, 0) = 1.0;
    state_transit_mat_(1, 1) = 1.0;
    state_transit_mat_(2, 2) = 1.0;
    state_transit_mat_(3, 3) = 1.0;
    state_transit_mat_(0, 2) = kTimeTick * std::cos(theta);
    state_transit_mat_(0, 3) = -vel * kTimeTick * std::sin(theta);
    state_transit_mat_(1, 2) = kTimeTick * std::sin(theta);
    state_transit_mat_(1, 3) = kTimeTick * vel * std::cos(theta);
    state_transit_mat_(3, 2) = kTimeTick * std::tan(delta) / kWheelBase;
    
    // Matrix B
    control_transit_mat_(3, 0) = kTimeTick * vel / (kWheelBase * std::cos(delta) * std::cos(delta));

    // Matrix C
    const_transit_mat_(0, 0) = kTimeTick * vel * std::sin(theta) * theta;
    const_transit_mat_(0, 1) = - kTimeTick * vel * std::cos(theta) * theta;
    const_transit_mat_(0, 3) = vel * delta / (kWheelBase * std::cos(delta) * std::cos(delta));
}




double* MpcControl::Predict(VehicleState *vehicle_state, autopilot_control_msgs::PathTrackByMpc::Request *req)
{
    
    double *x_ = new double[9];

    // state_mat_(:, 0) assignment
    Eigen::Vector4d temp_vec;
    temp_vec << vehicle_state->x, vehicle_state->y, vehicle_state->v, vehicle_state->theta;
    state_mat_.col(0) = temp_vec;

    // TODO Calculate transition matrix
    for (int i = 0; i < kHorizonLength; i++) {
        CalLinearModelMat(oper_path_(2, i), oper_path_(3, i), 0.0);  // TODO Add reference steering angle
        
        // Add constraints
        AddTransitConstraints(oper_path_(0, i), oper_path_(0, i+1),
                              oper_path_(1, i), oper_path_(1, i+1),
                              oper_path_(3, i), oper_path_(3, i+1),
                              oper_path_(2, i), x_[i]);

        // TODO add cost function and other constraints
    }
    
   return x_; 
}


void MpcControl::AddTransitConstraints(double x, double xx,
                                       double y, double yy,
                                       double theta, double theta_t,
                                       double v, double &delta)
{   /* 
    // Constraints of position x
    flow_[nef_count_] = fupp_[nef_count_] = x + v * std::cos(theta) * kTimeTick;
    f_[nef_count_++] = xx;

    // Constraints of position y
    flow_[nef_count_] = fupp_[nef_count_] = y + v * std::sin(theta) * kTimeTick;
    f_[nef_count_++] = yy;

    // Constraints of position theta
    flow_[nef_count_] = fupp_[nef_count_] = theta + v * std::tan(delta) / kWheelBase * kTimeTick;
    f_[nef_count_++] = theta_t; 
    */
}





