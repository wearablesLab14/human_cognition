#ifndef human_cognition_QNODE_LISTENER_HPP_
#define human_cognition_QNODE_LISTENER_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include "../node_common/qnode.hpp"
#endif

/**
 *
 */
class QNodeListener: public QNode {

public:
	QNodeListener(int argc, char** argv);
	virtual ~QNodeListener() {
	}

	/***********************************************
	 IMPLEMENTATION OF PURE VIRTUAL METHODS
	 ***********************************************/
	void startAction();
	void stopAction();
	void run();
	bool readyForAction();

	/***********************************************
	 SETTER
	 ***********************************************/
	virtual void setDisplayCoordinatesSignal(const bool &boolean);
	virtual void setDisplayCoordinatesFrame(const int &frame_index);
	virtual void setRecordCoordinatesSignal(const bool &boolean);
	virtual void setRecordCoordinatesFile(std::string file_name);

private:
	bool display_coordinates_signal;
	int display_coordinates_frame;
	bool record_coordinates_signal;
	std::string record_coordinates_file;

	tf::Vector3 tf_joint_coordinates[NUMBER_OF_FRAMES];
	tf::StampedTransform tf_message[NUMBER_OF_FRAMES];

};

#endif
