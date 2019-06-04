
/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <std_msgs/String.h>
#include "../include/gacui/subnode.h"

/*****************************************************************************
** Implementation
*****************************************************************************/

SubNode::SubNode(int argc, char** argv) :
    init_argc(argc),
    init_argv(argv)
    {}

SubNode::~SubNode() {
    if(ros::isStarted()) {
        ros::shutdown();  // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

/**
 * Callback function(void).
 * Construct the callback funciton in an object in order to maintain the
 * good packaging performance of the class.
 */
void SubNode::chatter_callback(const std_msgs::String::ConstPtr& msg) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;

    logging_model_msg << " [" << ros::Time::now() << "]: " << msg->data.c_str();

    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit loggingUpdated();
}

bool SubNode::init() {
    ros::init(init_argc, init_argv, "subnode");
    if( ! ros::master::check() ) {
        return false;
    }
    ros::start();
    ros::NodeHandle nh;
    chatter_subscriber = nh.subscribe("chatter", 1000, &SubNode::chatter_callback, this);
    // This function will automatically call SubNode.run()
    start();

    return true;
}

void SubNode::run() {

    while(ros::ok()) {
        ros::spinOnce();
    }
    std::cout << "SubNode shutdown, proceeding to close the gui." << std::endl;
    emit rosShutdown();
}
