#include "mpc_control.h"

int main()
{
    MpcControl *mpc_control = new MpcControl();

    const int kDaysInWeek = 7;  // 常量命名规范

    delete mpc_control;
    
    return 0;
}