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
	virtual ~QNodeListener() {}

	void startThread();
	void stopThread();
	void run();

private:
	tf::Vector3 vec[NUMBER_OF_FRAMES];
	tf::StampedTransform echo_transform[NUMBER_OF_FRAMES];

};

#endif
