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
	virtual ~QNodeReceiver() {}

	void startThread();
	void stopThread();
	void run();

private:
	void secondsToTime();
	void eulerAverage();
	double frame_euler_x;
	double frame_euler_y;
	double frame_euler_z;
	double frame_euler_x_average;
	double frame_euler_y_average;
	double frame_euler_z_average;

};

#endif
