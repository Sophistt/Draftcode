#ifndef MPCONTROLLER_H
#define MPCONTROLLER_H

#include <iostream>

class MPController
{
private:
    /* data */
    double *F;
    double *x;
public:
    MPController(/* args */);
    ~MPController();
};

#endif

