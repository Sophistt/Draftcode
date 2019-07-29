
#include <cstdlib>
#include "ros_node.h"

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "ros_node");

    // Initialize the RosNode class
    RosNode * rosNode = new RosNode();

    // Multiple threads
    ros::AsyncSpinner() spinner(2);
    spinner.start();
    ros::waitForShutdown();

    delete rosNode;
    
    return 0;
}
