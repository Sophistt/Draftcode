#ifndef MPC_CONTROL_H
#define MPC_CONTROL_H

#include <iostream>

class MpcControl
{
private:
    /* data */
    double *x_;
    double *f_;
public:
    MpcControl(/* args */);
    ~MpcControl();
};


#endif

