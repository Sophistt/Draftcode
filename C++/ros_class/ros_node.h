#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "ros/ros.h"
#include <boost/thread.hpp>
#include "std_msgs/Float32.h"


class RosNode {

public:
    RosNode();
    ~RosNode();

    void pathSubCallback(const std_msgs::Float32::ConstPtr & pathMsg);

private:
    ros::NodeHandle nh;

    ros::Subscriber pathSub;
    ros::Publisher anglePub;
    
    std_msgs::Float32 angle;

};

#endif
