
#include "ros_node.h"

RosNode::RosNode() {
    // Subscriber initialization
    pathSub = nh.subscribe("path_points", 1, &RosNode::pathSubCallback, this);
    // Publisher initialization
    anglePub = nh.advertise<std_msgs::Float32>("/control/steering_angle", 1);
}

RosNode::~RosNode() {

}

void RosNode::pathSubCallback(const std_msgs::Float32::ConstPtr & pathMsg) {
    double point = pathMsg->data;
    
    ROS_INFO("Receive Point: %lf", point);

    angle.data = point;

    anglePub.publish(angle);

    return;
}
