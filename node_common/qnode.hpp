#ifndef NODE_HPP_
#define NODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

//ROS includes
#include <ros/network.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <urdf/model.h>

//QT includes
#include <QStringList>
#include <QThread>
#include <QString>
#include <QColor>
#include <QFont>
#include <QStandardItemModel>
#include <QSettings>
#include <QTime>

//other includes
#include <ctime>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <boost/date_time/posix_time/posix_time.hpp>

//macros
#define PORT 5050
#define ONE_SEC_IN_NSEC 1000000000
#define NUMBER_OF_FRAMES 14
#define TO_ASSIGN_ADDRESS "000.000.0.000_0"
#define TO_IGNORE_ADDRESS "---.---.-.---_-"

//display type enums
enum DisplayType {
	TIP,
	INFO,
	ERROR,
	EULER,
	INACTIVE,
	ASYNCH,
	FRAME01,
	FRAME234,
	FRAME567,
	FRAME8910,
	FRAME111213,
	CALIBRATION
};

//struct for sensor packet data
struct SensorData {
	uint32_t timestamp;
	float q0, q1, q2, q3, exInt, eyInt, ezInt;
}__attribute__((packed));

/*! \brief Base class to setup a ROS node
 * @author Christian Benz <zneb_naitsirhc@web.de>
 * @author Christoph Döringer <christoph.doeringer@gmail.com>
 * @author Hendrik Pfeifer <hendrikpfeifer@gmail.com>
 * @author Heiko Reinemuth <heiko.reinemuth@gmail.com>
 */
class QNode: public QThread {
Q_OBJECT

public:
	QNode(int argc, char** argv, const std::string &name);
	virtual ~QNode();

	/***********************************************
	 PURE VIRTUAL METHODS
	 ***********************************************/
	virtual bool readyForAction() = 0;
	virtual void startAction() = 0;
	virtual void stopAction() = 0;
	virtual void run() = 0;

	/***********************************************
	 BASIC METHODS
	 ***********************************************/
	bool initNode();
	void shutdownNode();
	QString getFrameString(const int &frame_index);
	DisplayType getFrameDisplayType(const int &frame_index);
	void display(const DisplayType &level, const QString &info);
	std::string rosTimeToLocalTime(ros::Time stamp);

	/***********************************************
	 GETTER METHODS
	 ***********************************************/
	QStandardItemModel* getListViewModel();

Q_SIGNALS:
	/***********************************************
	 SIGNALS TO GUI
	 ***********************************************/
	void listViewModelUpdated();
	void frameDataUpdated();
	void calibrationSwitchUpdated();

private:
	/***********************************************
	 PRIVATE HELPER METHODS
	 ***********************************************/
	std::string getIP4HostAddress();

protected:
	int init_argc;
	char** init_argv;

	//node name
	const std::string node_name;

	//standard listView model for nodes
	QStandardItemModel listViewModel;

	//urdf link names
	std::string baseLinkName;
	std::string frameLinkName[NUMBER_OF_FRAMES];

	//urdf joint names
	std::string baseJointName;
	std::string frameJointName[NUMBER_OF_FRAMES];
};

#endif
