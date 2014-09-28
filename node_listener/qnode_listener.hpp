#ifndef human_cognition_QNODE_LISTENER_HPP_
#define human_cognition_QNODE_LISTENER_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include "../node_common/qnode.hpp"
#endif

/*! \brief Specialized node class for listening to ROS tf messages
 * @author Christian Benz <zneb_naitsirhc@web.de>
 * @author Christoph Döringer <christoph.doeringer@gmail.com>
 * @author Hendrik Pfeifer <hendrikpfeifer@gmail.com>
 * @author Heiko Reinemuth <heiko.reinemuth@gmail.com>
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
	virtual void setSignalCoordinates(const bool &boolean);
	virtual void setFrameCoordinates(const int &frame_index);
	virtual void setSignalRecord(const bool &boolean);
	virtual void setFileRecord(std::string file_name);

private:
	/***********************************************
	 PRIVATE HELPER METHODS
	 ***********************************************/
	void displayCoordinates();

	//settings variables
	bool signalCoordinates;
	int frameCoordinates;
	bool signalRecord;
	std::string fileRecord;

	//absolute coordinates of a frame's parent joint
	tf::Vector3 absoluteJointCoordinates[NUMBER_OF_FRAMES];

	//frame message
	tf::StampedTransform frameMsg[NUMBER_OF_FRAMES];
};

#endif
