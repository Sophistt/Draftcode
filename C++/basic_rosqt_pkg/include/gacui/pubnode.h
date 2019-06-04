/**
 * @file /include/testgui/PubNode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PUBNODE_HPP_
#define PUBNODE_HPP_

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
#include <QStringListModel>


/*****************************************************************************
** Class
*****************************************************************************/

class PubNode : public QThread {
    Q_OBJECT
public:
        PubNode(int argc, char** argv);
  virtual ~PubNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

        /*********************
        ** Logging
        **********************/
        enum LogLevel {
                 Debug,
                 Info,
                 Warn,
                 Error,
                 Fatal
         };

	QStringListModel* loggingModel() { return &logging_model; }
  void log(const LogLevel &level, const std::string &msg);

signals:
    void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
  QStringListModel logging_model;
};

#endif /* PUbNODE_HPP_ */
