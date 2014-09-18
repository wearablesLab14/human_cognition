#ifndef NODE_HPP_
#define NODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <ros/network.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <QStringList>
#include <QThread>
#include <QString>
#include <QColor>
#include <QFont>
#include <QStandardItemModel>
#include <QSettings>
#include <QTime>

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <ifaddrs.h>

#define PORT 5050
#define NUMBER_OF_FRAMES 14
#define TO_ASSIGN_ADDRESS "000.000.0.000"
#define TO_IGNORE_ADDRESS "---.---.-.---"

enum DisplayType {
	INSTRUCTION, INFO, ERROR, FRAME01, FRAME234, FRAME567, FRAME8910, FRAME111213
};

struct SensorData {
	uint32_t timestamp;
	float q0, q1, q2, q3, exInt, eyInt, ezInt;
}__attribute__((packed));

/**
 *
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
	DisplayType getLevelForFrame(const int &frame_index);
	void display(const DisplayType &level, const QString &info);

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

private:
	/***********************************************
	 PRIVATE HELPER METHODS
	 ***********************************************/
	std::string findHostAddress();

protected:
	int init_argc;
	char** init_argv;
	const std::string node_name;

	QStandardItemModel list_view_model;
	std::string link_name[NUMBER_OF_FRAMES];
};

#endif
