/*================================================================
*   Copyright (C) 2020 Sangfor Ltd. All rights reserved.
*   
*   @file:mpc_server.cpp
*   @author: Sophistt
*   @date:2020-05-18 14:20
*   @description: 
*
================================================================*/


#include "mpc_control.h"

// For test
static double c1, c2;
static double *x_;

// Mpc object 
static MpcControl *mpc_control = new MpcControl();

// snOptA object
static snoptProblemA sp;
// snopt Variables
const int kCold = 0, kBasis = 1, kWarm = 2;
static int n, neF, nS, nInf, cnF;
static double sInf;

static double *x, *xlow, *xupp, *xmul;
static double *F, *Flow, *Fupp, *Fmul;
static int *xstate, *Fstate;

static double objadd;
static int objrow;

// Define cost function constraints in this function.
void SolveUsrF(int *Status, int *n, double x[],
               int  *needF, int *neF, double F[], 
               int  *needG, int *neG, double G[],
               char    *cu, int *lencu,
               int    iu[], int *leniu,
               double ru[], int *lenru)
{
    // In this function, redefine the constraints
    F[0] = x[1];
    F[1] = x[0] * x[0] + 4 * x[1] * x[1];
    F[2] = (x[0] - 2) * (x[0] - 2) + x[1] * x[1];
};


void SnOptInit()
{
    // SnOpt Init.
    sp.initialize("", 1);
    sp.setIntParameter("Derivative", 0);
    sp.setIntParameter("Verify level", 3);

    // Snopt Variables Init.
    // n = mpc_control->GetHorizontLength(); neF = 3; nS = 0;
    n = 2; neF = 3; nS = 0;  // For test

    x = new double[n]; xlow = new double[n]; xupp = new double[n]; xmul = new double[n];
    F = new double[neF]; Flow = new double[neF]; Fupp = new double[neF]; Fmul = new double[neF];
    xstate = new int[n]; Fstate = new int[neF];

    objadd = 0;
    objrow = 0;

};


bool MpcCallback(snopt_msgs::SnoptCal::Request &req,
                 snopt_msgs::SnoptCal::Response &res)
{
    c1 = req.c1;
    c2 = req.c2;

    // Set boundary
    xlow[0] = 0.0; xlow[1] = -1e20;
    xupp[0] = 1e20; xupp[1] = 1e20;
    xstate[0] = 0; xstate[1] = 0;
    
    Flow[0] = -1e20; Flow[1] = -1e20; Flow[2] = -1e20;
    Fupp[0] = 1e20;  Fupp[1] = 4.0;   Fupp[2] = 5.0;
    Fmul[0] = 0;     Fmul[1] = 0;     Fmul[2] = 0;
    x[0] = 1.0; x[1] = 1.0;
   
    sp.solve(kCold, neF, n, objadd, objrow, SolveUsrF,
            xlow, xupp, Flow, Fupp,
            x, xstate, xmul, F, Fstate, Fmul,
            nS, nInf, sInf);
    
    res.x.clear();
    res.x.push_back(*(x));
    res.x.push_back(*(x + 1));

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("mpc", MpcCallback);
    
    // Static Variables Init
    c1 = c2 = 0;
    SnOptInit();

    ROS_INFO("Ready to MPC Control");
    ros::spin();
    
    return 0;
}

