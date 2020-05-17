#include <stdlib.h>
#include <iostream>
#include <Eigen\Dense>

using Eigen::MatrixXd;

int main()
{
    MatrixXd m = MatrixXd::Random(3, 3);
    m = (m + MatrixXd::Constant(3, 3, 1.2)) * 50;
    std::cout << "m = " << std::endl << m << std::endl;

    Eigen::VectorXd v(3);
    v << 1, 2, 3;
    std::cout << "m * v = " << std::endl << m * v << std::endl;
    
    return 0;
}

