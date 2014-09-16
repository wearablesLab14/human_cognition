#include "qnode_receiver.hpp"

/**
 *
 * @param argc
 * @param argv
 */
QNodeReceiver::QNodeReceiver(int argc, char** argv) :
		QNode(argc, argv, "receiver") {

	display(INSTRUCTION, "1. Connect to mocap");
	display(INSTRUCTION, "2. Start roscore process");
	display(INSTRUCTION, "3. Start rviz");

	frame_euler_x = 0;
	frame_euler_y = 0;
	frame_euler_z = 0;
	frame_euler_x_average = 0;
	frame_euler_y_average = 0;
	frame_euler_z_average = 0;
}

/**
 *
 */
void QNodeReceiver::startThread() {
	start();
}

/**
 *
 */
void QNodeReceiver::stopThread() {
	rosShutdown();
}





/**
 *
 */
void QNodeReceiver::secondsToTime() {

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		int h = (frame_inactivity_sec[i] / 60 / 60) % 24;
		int m = (frame_inactivity_sec[i] / 60) % 60;
		int s = frame_inactivity_sec[i] % 60;
		QTime time(h, m, s);
		frame_inactivity[i] = time.toString();
	}
}


/**
 *
 */
void QNodeReceiver::eulerAverage() {

	frame_euler_x_average /= frame_hertz[display_euler_frame];
	frame_euler_y_average /= frame_hertz[display_euler_frame];
	frame_euler_z_average /= frame_hertz[display_euler_frame];

	display(INFO,
			QString("%1, %2, %3").arg(frame_euler_x_average).arg(
					frame_euler_y_average).arg(frame_euler_z_average));

	frame_euler_x_average = 0;
	frame_euler_y_average = 0;
	frame_euler_z_average = 0;
}

/**
 *
 */
void QNodeReceiver::run() {

	frame_euler_x = 0;
	frame_euler_y = 0;
	frame_euler_z = 0;
	frame_euler_x_average = 0;
	frame_euler_y_average = 0;
	frame_euler_z_average = 0;

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		frame_hertz[i] = 0;
		frame_inactivity[i] = QTime(0, 0, 0).toString();
	}

	tf::TransformBroadcaster tfPublisher;

	int x = 0;

	while (ros::ok() && x < 10000) {

		for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

			tf_message[i].setRotation(tf_rotation[i]);
			tf_message[i].stamp_ = ros::Time::now();
			tf_last_message_stamp[i] = tf_message[i].stamp_;
			tfPublisher.sendTransform(tf_message[i]);
		}
		x++;
	}

	//frame rotation correction
	tf::Quaternion rotationYCorrection(0, sqrt(0.5), 0, -sqrt(0.5));
	rotationYCorrection.normalize();
	tf::Quaternion rotationZCorrection(0, 0, sqrt(0.5), -sqrt(0.5));
	rotationZCorrection.normalize();

	int currentFrame;
	QString currentAddress;

	display(INFO, QString("Receiving has started"));

	ros::Duration timeout(1.0); // Timeout of 2 seconds
	ros::Time start_time = ros::Time::now();

	//receive sensor packets while ROS is ok
	while (ros::ok()) {

		//receive bytes to fill struct
		sensor_packet_size = recvfrom(udp_socket, (char*) &sensor_packet_data,
				sizeof(sensor_packet_data), 0,
				(struct sockaddr *) &sensor_address, &sensor_address_length);

		//if bytes were received
		if (sensor_packet_size > 0) {

			currentAddress = QString(inet_ntoa(sensor_address.sin_addr));

			currentFrame = frame_ip_list.indexOf(currentAddress, 0);

			if (currentFrame < 0) {

				currentFrame = frame_ip_list.indexOf(ip_to_assign, 0);

				if (currentFrame >= 0) {
					setFrameAddress(currentFrame, currentAddress);
				}

			}

			if (currentFrame >= 0 && currentFrame < NUMBER_OF_FRAMES) {

				tf::Quaternion original(sensor_packet_data.q1,
						sensor_packet_data.q2, sensor_packet_data.q3,
						sensor_packet_data.q0);
				original.normalize();

				//correct original rotation
				if (currentFrame != 7 && currentFrame != 13) {
					original = original * rotationYCorrection;
				}
				original = original * rotationZCorrection;

				if (display_euler_signal && currentFrame == display_euler_frame) {
					tf::Matrix3x3(original).getRPY(frame_euler_x, frame_euler_y,
							frame_euler_z);
					frame_euler_x_average += frame_euler_x;
					frame_euler_y_average += frame_euler_y;
					frame_euler_z_average += frame_euler_z;
				}

				//rotation[currentFrame] = tf::Quaternion(original.getX(),
				//original.getY(), original.getZ(), original.getW());
				tf_rotation[currentFrame] = tf::Quaternion(original.getY(),
						-original.getX(), original.getZ(), original.getW());

				//*********************************************************************

				//kinematic
				if (currentFrame == 2 || currentFrame == 5 || currentFrame == 8
						|| currentFrame == 11) {
					tf_rotation[currentFrame] = inverse(tf_rotation[0])
							* tf_rotation[currentFrame];
				} else if (currentFrame != 0) {
					tf_rotation[currentFrame] = inverse(
							tf_rotation[currentFrame - 1])
							* tf_rotation[currentFrame];
				}

				tf_message[currentFrame].setRotation(tf_rotation[currentFrame]);

				frame_hertz[currentFrame] = frame_hertz[currentFrame] + 1;

				if (reset_model_signal) {
					resetRotation();
					for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
						tf_message[i].setRotation(tf_rotation[i]);
					}
					reset_model_signal = false;
				}

				tf_base_message.stamp_ = ros::Time::now();
				tfPublisher.sendTransform(tf_base_message);

				for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

					tf_message[i].stamp_ = ros::Time::now();

					tfPublisher.sendTransform(tf_message[i]);

					//update last update time for current frame
					if (i == currentFrame) {

						tf_last_message_stamp[currentFrame] =
								tf_message[currentFrame].stamp_;
					}

					//*********************************************************************

					//update data age (seconds)
					frame_inactivity_sec[i] = tf_message[i].stamp_.sec
							- tf_last_message_stamp[i].sec;
				}

				if ((ros::Time::now() - start_time) >= timeout) {

					if (display_euler_signal && frame_hertz[display_euler_frame] != 0) {
						eulerAverage();
					}

					secondsToTime();
					Q_EMIT frameInfoUpdated();
					start_time = ros::Time::now();
				}
			}
		}
	}
	display(INFO, QString("Receiving has stopped"));
}

//**************************************************************************************
