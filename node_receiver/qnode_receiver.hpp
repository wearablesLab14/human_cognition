#ifndef human_cognition_QNODE_RECEIVER_HPP_
#define human_cognition_QNODE_RECEIVER_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include "../node_common/qnode.hpp"
#endif

/**
 *
 */
class QNodeReceiver: public QNode {

public:
	QNodeReceiver(int argc, char** argv);
	virtual ~QNodeReceiver() {
	}

	/***********************************************
	 IMPLEMENTATION OF PURE VIRTUAL METHODS
	 ***********************************************/
	bool readyForAction();
	void startAction();
	void stopAction();
	void run();

	/***********************************************
	 GETTER
	 ***********************************************/
	virtual QString getAssignAddress();
	virtual QString getIgnoreAddress();
	virtual QString getFrameAddress(const int &frame_index);
	virtual int getFrameHertz(const int &frame_index);
	virtual QString getFrameInactivity(const int &frame_index);

	/***********************************************
	 SETTER
	 ***********************************************/
	virtual void clearAllFrameAddresses();
	virtual void addFrameAddress(QString address);
	virtual void setFrameAddress(const int &frame_index, QString address);
	virtual void setFrameHertz(const int &frame_index, const int &value);
	virtual void setFrameInactivity(const int &frame_index, QString time);
	virtual void setLastUpdate(const int &frame_index, const ros::Time &time);
	virtual void setResetModelSignal(const bool &boolean);
	virtual void setDisplayEulerSignal(const bool &boolean);
	virtual void setDisplayEulerFrame(const int &frame_index);

private:
	/***********************************************
	 PRIVATE HELPER METHODS
	 ***********************************************/
	bool socketReady();
	bool socketCreation();
	bool socketOption();
	bool socketBinding();
	void getTimeFromSec();
	void displayEulerAverage();
	void resetFrameRotation();
	void resetEuler();
	void resetFrameData();

	int udp_socket;
	int udp_socket_binding;
	int udp_socket_option;
	struct sockaddr_in socket_address;
	struct sockaddr_in sensor_address;
	socklen_t sensor_address_length;
	SensorData sensor_packet_data;
	int sensor_packet_size;

	double frame_euler_x;
	double frame_euler_y;
	double frame_euler_z;
	double frame_euler_x_average;
	double frame_euler_y_average;
	double frame_euler_z_average;

	bool display_euler_signal;
	int display_euler_frame;

	bool reset_model_signal;

	QString assign_address;
	QString ignore_address;

	QStringList frame_address_list;
	int frame_inactivity_sec[NUMBER_OF_FRAMES];
	QString frame_inactivity[NUMBER_OF_FRAMES];
	ros::Time tf_last_message_stamp[NUMBER_OF_FRAMES];
	int frame_hertz[NUMBER_OF_FRAMES];

	tf::Quaternion tf_base_link_rotation;
	tf::StampedTransform tf_base_link_message;

	std::string tf_link_parent[NUMBER_OF_FRAMES];
	std::string tf_link_child[NUMBER_OF_FRAMES];
	tf::Vector3 tf_joint_origin[NUMBER_OF_FRAMES];
	tf::Quaternion tf_rotation[NUMBER_OF_FRAMES];
	tf::StampedTransform tf_message[NUMBER_OF_FRAMES];
};

#endif
