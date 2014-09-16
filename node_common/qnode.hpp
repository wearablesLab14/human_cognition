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

enum DisplayLevel {
	INSTRUCTION, INFO, ERROR, ACTIVE_FRAME
};

struct SensorData {
	uint32_t timestamp;
	float q0, q1, q2, q3, exInt, eyInt, ezInt;
}__attribute__((packed));

class QNode : public QThread {
Q_OBJECT

public:
	QNode(int argc, char** argv, const std::string &name);
	virtual ~QNode();
	virtual void startThread() = 0;
	virtual void stopThread() = 0;
	virtual void run() = 0;

	void rosShutdown();
	std::string findHostAddress();
	bool nodeReady();
	bool receiveReady();
	bool listenReady();
	bool socketReady();
	bool socketCreation();
	bool socketOption();
	bool socketBinding();
	void display(const DisplayLevel &level, const QString &info);
	void initMessages();
	void initAddresses();
	void resetRotation();

	/***********************************************
	GETTER
	***********************************************/
	QStandardItemModel* getListViewModel();
	QString getAssignAddress();
	QString getIgnoreAddress();
	QString getFrameAddress(const int &frame_index);
	int getFrameHertz(const int &frame_index);
	QString getFrameInactivity(const int &frame_index);

	/***********************************************
	SETTER
	***********************************************/
	void clearAllFrameAddesses();
	void addFrameAddress(QString address);
	void setFrameAddress(const int &frame_index, QString address);
	void setFrameHertz(const int &frame_index, const int &value);
	void setFrameInactivity(const int &frame_index, QString time);
	void setLastUpdate(const int &frame_index, const ros::Time &time);
	void setResetModelSignal(const bool &boolean);
	void setDisplayEulerSignal(const bool &boolean);
	void setDisplayEulerFrame(const int &frame_index);
	void setDisplayCoordinatesSignal(const bool &boolean);
	void setDisplayCoordinatesFrame(const int &frame_index);
	void setRecordCoordinatesSignal(const bool &boolean);
	void setRecordCoordinatesFile(std::string file_name);

Q_SIGNALS:
	void listInfoUpdated();
	void frameInfoUpdated();

protected:
	int init_argc;
	char** init_argv;
	const std::string node_name;
	QStandardItemModel list_view_model;

	bool reset_model_signal;
	bool display_euler_signal;
	int display_euler_frame;
	bool display_coordinates_signal;
	int display_coordinates_frame;
	bool record_coordinates_signal;
	std::string record_coordinates_file;

	QString ip_to_assign;
	QString ip_to_ignore;
	QStringList frame_ip_list;

	int frame_inactivity_sec[NUMBER_OF_FRAMES];
	QString frame_inactivity[NUMBER_OF_FRAMES];
	ros::Time tf_last_message_stamp[NUMBER_OF_FRAMES];
	int frame_hertz[NUMBER_OF_FRAMES];

	tf::Quaternion tf_base_rotation;
	tf::StampedTransform tf_base_message;

	std::string tf_links[NUMBER_OF_FRAMES];
	std::string tf_link_parent[NUMBER_OF_FRAMES];
	std::string tf_link_child[NUMBER_OF_FRAMES];
	tf::Vector3 tf_joint_origin[NUMBER_OF_FRAMES];
	tf::Quaternion tf_rotation[NUMBER_OF_FRAMES];
	tf::StampedTransform tf_message[NUMBER_OF_FRAMES];

	int udp_socket;
	int udp_socket_binding;
	int udp_socket_option;
	struct sockaddr_in socket_address;
	struct sockaddr_in sensor_address;
	socklen_t sensor_address_length;
	SensorData sensor_packet_data;
	int sensor_packet_size;

};

#endif
