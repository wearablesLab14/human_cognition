#include "qnode_listener.hpp"

#include <iostream>
#include <fstream>
#include <stdio.h>

class echoListener {
public:

	tf::TransformListener tf;

//constructor with name
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
}

/**
 *
 */
void QNodeListener::startThread() {
	start();
}

/**
 *
 */
void QNodeListener::stopThread() {
	rosShutdown();
}

/**
 *
 */
void QNodeListener::run() {


	echoListener echoListener;
	std::ofstream coordinates;

	if (display_coordinates_signal) {
	coordinates.open(record_coordinates_file.c_str(), std::ios::out | std::ios::app);
	}


	//base_link is world coordinate frame
	std::string source_frameid = "/base_link";
	//joint coordinate frame you want to have coordinates from (relative to world frame)

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		echoListener.tf.waitForTransform(source_frameid, tf_links[i],
				ros::Time(0), ros::Duration(1.0));
	}

	ros::Duration timeout(1.0);
	ros::Time start_time = ros::Time::now();

	while (ros::ok()) {

		try {

			for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
				echoListener.tf.lookupTransform(source_frameid, tf_links[i],
						ros::Time(0), echo_transform[i]);
			}

			for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
				vec[i] = echo_transform[i].getOrigin();
			}

			if (display_coordinates_signal) {

			for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
				coordinates << i << ";" << vec[i].getX() << ";"
						<< vec[i].getY() << ";" << vec[i].getZ() << "\n";
			}
			}


			if (display_coordinates_signal && (ros::Time::now() - start_time) >= timeout) {

				QString msg(tf_links[display_coordinates_frame].c_str());
				msg.append(QString(" x: "));
				msg.append(QString::number(vec[display_coordinates_frame].getX()));
				msg.append(QString(" y: "));
				msg.append(QString::number(vec[display_coordinates_frame].getY()));
				msg.append(QString(" z: "));
				msg.append(QString::number(vec[display_coordinates_frame].getZ()));

				display(ACTIVE_FRAME, msg);

				start_time = ros::Time::now();
			}

		} catch (tf::TransformException& ex) {

			display(INSTRUCTION, QString(ex.what()));
			display(INSTRUCTION,
					QString::fromStdString(
							echoListener.tf.allFramesAsString()));

		}
	}
	if (display_coordinates_signal) {
	coordinates.close();
	}


	display(INFO, QString("listening has stopped"));
}

//**************************************************************************************
