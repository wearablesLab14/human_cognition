#include "qnode_listener.hpp"

#include <iostream>
#include <fstream>
#include <stdio.h>

/**
 *
 */
class echoListener {
public:
	tf::TransformListener tf;
	echoListener() {
	}
	;

	~echoListener() {
	}
	;

private:
};

/**
 *
 * @param argc
 * @param argv
 */
QNodeListener::QNodeListener(int argc, char** argv) :
		QNode(argc, argv, "listener") {

	display(TIP, "1. Connect to mocap");
	display(TIP, "2. Start receiver");

	display_coordinates_signal = false;
	display_coordinates_frame = 0;

	record_coordinates_signal = false;
	record_coordinates_file = "coordinates.csv";
}

/***********************************************
 IMPLEMENTATION OF PURE VIRTUAL METHODS
 ***********************************************/

/**
 *
 * @return
 */
bool QNodeListener::readyForAction() {

	if (!initNode()) {
		return false;
	}
	if (ros::isStarted()) {
		display(INFO, QString("Ready to listen"));
	}
	return true;
}

/**
 *
 */
void QNodeListener::startAction() {
	start();
}

/**
 *
 */
void QNodeListener::stopAction() {
	shutdownNode();
}

/**
 *
 */
void QNodeListener::run() {

	echoListener echoListener;
	std::ofstream coordinates;

	if (record_coordinates_signal) {
		coordinates.open(record_coordinates_file.c_str(),
				std::ios::out | std::ios::app);
	}

	std::string source_frameid = "/base_link";

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

		if (echoListener.tf.waitForTransform(source_frameid, frameLinkName[i],
				ros::Time(0), ros::Duration(0.3))) {
			display(getFrameDisplayType(i),
					getFrameString(i).append(QString(" found")));
		} else {
			display(getFrameDisplayType(i),
					getFrameString(i).append(QString(" not found")));
		}
	}

	ros::Duration timeout(1.0);
	ros::Time start_time = ros::Time::now();

	int oldnsec = 0;

	while (ros::ok()) {

		try {

			for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
				echoListener.tf.lookupTransform(source_frameid, frameLinkName[i],
						ros::Time(0), tf_message[i]);
			}

			if (tf_message[0].stamp_.nsec != oldnsec) {

				oldnsec = tf_message[0].stamp_.nsec;

				for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
					tf_joint_coordinates[i] = tf_message[i].getOrigin();
				}

				if (record_coordinates_signal) {

					for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
						coordinates << i << ";"
								<< rosTimeToTimezone(tf_message[i].stamp_, 2)
								<< ";" << tf_joint_coordinates[i].getX() << ";"
								<< tf_joint_coordinates[i].getY() << ";"
								<< tf_joint_coordinates[i].getZ() << "\n";
					}
				}
			}

			if ((ros::Time::now() - start_time) >= timeout) {

				if (display_coordinates_signal) {

					QString msg(frameLinkName[display_coordinates_frame].c_str());
					msg.append(QString(" x: "));
					msg.append(
							QString::number(
									tf_joint_coordinates[display_coordinates_frame].getX()));
					msg.append(QString(" y: "));
					msg.append(
							QString::number(
									tf_joint_coordinates[display_coordinates_frame].getY()));
					msg.append(QString(" z: "));
					msg.append(
							QString::number(
									tf_joint_coordinates[display_coordinates_frame].getZ()));

					display(getFrameDisplayType(display_coordinates_frame),
							msg);
				}

				if (record_coordinates_signal) {
					display(INFO, QString("Recording coordinates"));
				}

				start_time = ros::Time::now();
			}

		} catch (tf::TransformException& ex) {

		}
	}
	if (record_coordinates_signal) {
		coordinates.close();
	}
	display(INFO, QString("Listening has stopped"));
}

/***********************************************
 SETTER
 ***********************************************/

/**
 *
 * @param boolean
 */
void QNodeListener::setDisplayCoordinatesSignal(const bool &boolean) {
	display_coordinates_signal = boolean;
}

/**
 *
 * @param frame_index
 */
void QNodeListener::setDisplayCoordinatesFrame(const int &frame_index) {
	display_coordinates_frame = frame_index;
}

/**
 *
 * @param boolean
 */
void QNodeListener::setRecordCoordinatesSignal(const bool &boolean) {
	record_coordinates_signal = boolean;
}

/**
 *
 * @param file
 */
void QNodeListener::setRecordCoordinatesFile(std::string file) {
	record_coordinates_file = file;
}
