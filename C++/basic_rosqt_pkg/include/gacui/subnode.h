/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef SUBNODE_H
#define SUBNODE_H

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <std_msgs/String.h>
#include <QStringListModel>


/*****************************************************************************
** Class
*****************************************************************************/

class SubNode : public QThread
{
  Q_OBJECT

public:
    SubNode(int argc, char** argv);
    virtual ~SubNode();
    bool init();
    void run();


    QStringListModel* loggingModel() { return &logging_model; }
    void chatter_callback(const std_msgs::String::ConstPtr& msg);


signals:
    void loggingUpdated();
    void rosShutdown();

private:
    int init_argc;
    char** init_argv;
    ros::Subscriber chatter_subscriber;
    QStringListModel logging_model;

};


#endif // SUBNODE_H
