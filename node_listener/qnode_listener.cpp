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

	display(INSTRUCTION, "1. Connect to mocap");
	display(INSTRUCTION, "2. Start receiver");

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
		echoListener.tf.waitForTransform(source_frameid, link_name[i],
				ros::Time(0), ros::Duration(1.0));
	}

	ros::Duration timeout(1.0);
	ros::Time start_time = ros::Time::now();

	while (ros::ok()) {

		try {

			for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
				echoListener.tf.lookupTransform(source_frameid, link_name[i],
						ros::Time(0), tf_message[i]);
			}

			for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
				tf_joint_coordinates[i] = tf_message[i].getOrigin();
			}

			if (record_coordinates_signal) {

				for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
					coordinates << i << ";" << tf_joint_coordinates[i].getX()
							<< ";" << tf_joint_coordinates[i].getY() << ";"
							<< tf_joint_coordinates[i].getZ() << "\n";
				}
			}

			if (display_coordinates_signal
					&& (ros::Time::now() - start_time) >= timeout) {

				QString msg(link_name[display_coordinates_frame].c_str());
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

				display(getLevelForFrame(display_coordinates_frame), msg);

				start_time = ros::Time::now();
			}

		} catch (tf::TransformException& ex) {

			display(INSTRUCTION, QString(ex.what()));
			display(INSTRUCTION,
					QString::fromStdString(
							echoListener.tf.allFramesAsString()));

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
