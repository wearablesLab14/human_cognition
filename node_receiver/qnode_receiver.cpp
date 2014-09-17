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

	udp_socket = -1;
	udp_socket_binding = -1;
	udp_socket_option = -1;
	sensor_address_length = 0;
	sensor_packet_size = 0;

	//set socket address to 0
	memset((char *) &socket_address, 0, sizeof(socket_address));

	//set sensor address to 0
	memset((char *) &sensor_address, 0, sizeof(sensor_address));

	//fill socket address
	socket_address.sin_family = AF_INET;
	socket_address.sin_addr.s_addr = htonl(INADDR_ANY);
	socket_address.sin_port = htons(PORT);

	//sensor address length
	sensor_address_length = sizeof(sensor_address);

	resetEuler();

	display_euler_signal = false;
	display_euler_frame = 0;

	reset_model_signal = false;

	assign_address = QString(TO_ASSIGN_ADDRESS);
	ignore_address = QString(TO_IGNORE_ADDRESS);

	resetFrameData();

	tf_base_link_rotation = tf::Quaternion(0, 0, 0, sqrt(1.0));
	tf_base_link_rotation.normalize();
	tf_base_link_message.frame_id_ = "base_link";
	tf_base_link_message.child_frame_id_ = "base_link_connector";
	tf_base_link_message.setOrigin(tf::Vector3(0, 0, 0.8));
	tf_base_link_message.setRotation(tf_base_link_rotation);

	//URDF Joints
	//************************************
	tf_joint_origin[0] = tf::Vector3(0, 0, 0);
	tf_joint_origin[1] = tf::Vector3(0, 0, 0.53);
	tf_link_parent[0] = "base_link_connector";
	tf_link_child[0] = link_name[0];
	tf_link_parent[1] = link_name[0];
	tf_link_child[1] = link_name[1];
	//************************************
	tf_joint_origin[2] = tf::Vector3(0, 0.3, 0.52);
	tf_joint_origin[3] = tf::Vector3(0, 0, -0.25);
	tf_joint_origin[4] = tf::Vector3(0, 0, -0.27);
	tf_link_parent[2] = link_name[0];
	tf_link_child[2] = link_name[2];
	tf_link_parent[3] = link_name[2];
	tf_link_child[3] = link_name[3];
	tf_link_parent[4] = link_name[3];
	tf_link_child[4] = link_name[4];
	//************************************
	tf_joint_origin[5] = tf::Vector3(0, 0.1, 0);
	tf_joint_origin[6] = tf::Vector3(0, 0, -0.37);
	tf_joint_origin[7] = tf::Vector3(0, 0, -0.43);
	tf_link_parent[5] = "base_link_connector";
	tf_link_child[5] = link_name[5];
	tf_link_parent[6] = link_name[5];
	tf_link_child[6] = link_name[6];
	tf_link_parent[7] = link_name[6];
	tf_link_child[7] = link_name[7];
	//************************************
	tf_joint_origin[8] = tf::Vector3(0, -0.3, 0.52);
	tf_joint_origin[9] = tf::Vector3(0, 0, -0.25);
	tf_joint_origin[10] = tf::Vector3(0, 0, -0.27);
	tf_link_parent[8] = link_name[0];
	tf_link_child[8] = link_name[8];
	tf_link_parent[9] = link_name[8];
	tf_link_child[9] = link_name[9];
	tf_link_parent[10] = link_name[9];
	tf_link_child[10] = link_name[10];
	//************************************
	tf_joint_origin[11] = tf::Vector3(0, -0.1, 0);
	tf_joint_origin[12] = tf::Vector3(0, 0, -0.37);
	tf_joint_origin[13] = tf::Vector3(0, 0, -0.43);
	tf_link_parent[11] = "base_link_connector";
	tf_link_child[11] = link_name[11];
	tf_link_parent[12] = link_name[11];
	tf_link_child[12] = link_name[12];
	tf_link_parent[13] = link_name[12];
	tf_link_child[13] = link_name[13];
	//************************************

	resetFrameRotation();

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

		//initialize TF messages
		tf_message[i].frame_id_ = tf_link_parent[i];
		tf_message[i].child_frame_id_ = tf_link_child[i];
		tf_message[i].setOrigin(tf_joint_origin[i]);
		tf_message[i].setRotation(tf_rotation[i]);
	}

}

/***********************************************
 IMPLEMENTATION OF PURE VIRTUAL METHODS
 ***********************************************/

/**
 *
 * @return
 */
bool QNodeReceiver::readyForAction() {

	if (!initNode()) {
		return false;
	}
	if (!socketReady()) {
		return false;
	}
	if (ros::isStarted()) {
		display(INFO, QString("Ready to receive"));
	}
	return true;
}

/**
 *
 */
void QNodeReceiver::startAction() {
	start();
}

/**
 *
 */
void QNodeReceiver::stopAction() {
	shutdownNode();
}

/**
 *
 */
void QNodeReceiver::run() {

	resetEuler();
	resetFrameData();

	tf::TransformBroadcaster tfPublisher;

	//frame rotation correction
	tf::Quaternion rotationYCorrection(0, sqrt(0.5), 0, -sqrt(0.5));
	rotationYCorrection.normalize();
	tf::Quaternion rotationZCorrection(0, 0, sqrt(0.5), -sqrt(0.5));
	rotationZCorrection.normalize();

	ros::Duration initial_timeout(2.0);
	ros::Time initial_start_time = ros::Time::now();

	while (ros::ok()
			&& ((ros::Time::now() - initial_start_time) < initial_timeout)) {

		tf_base_link_message.stamp_ = ros::Time::now();
		tfPublisher.sendTransform(tf_base_link_message);

		for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
			tf_message[i].stamp_ = ros::Time::now();
			tf_last_message_stamp[i] = tf_message[i].stamp_;
			tfPublisher.sendTransform(tf_message[i]);
		}
	}

	int currentFrame;
	QString currentAddress;

	display(INFO, QString("Receiving has started"));

	ros::Duration timeout(1.0);
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

			currentFrame = frame_address_list.indexOf(currentAddress, 0);

			if (currentFrame < 0) {

				currentFrame = frame_address_list.indexOf(assign_address, 0);

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

				if (display_euler_signal
						&& currentFrame == display_euler_frame) {
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
					resetFrameRotation();
					for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
						tf_message[i].setRotation(tf_rotation[i]);
					}
					reset_model_signal = false;
				}

				tf_base_link_message.stamp_ = ros::Time::now();
				tfPublisher.sendTransform(tf_base_link_message);

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

					if (display_euler_signal
							&& frame_hertz[display_euler_frame] != 0) {
						displayEulerAverage();
					}

					getTimeFromSec();

					Q_EMIT frameDataUpdated();

					start_time = ros::Time::now();
				}
			}
		}
	}
	display(INFO, QString("Receiving has stopped"));
}

/***********************************************
 GETTER
 ***********************************************/

/**
 *
 * @return
 */
QString QNodeReceiver::getAssignAddress() {
	return assign_address;
}

/**
 *
 * @return
 */
QString QNodeReceiver::getIgnoreAddress() {
	return ignore_address;
}

/**
 *
 * @param frame_index
 * @return
 */
QString QNodeReceiver::getFrameAddress(const int &frame_index) {
	return frame_address_list.at(frame_index);
}

/**
 *
 * @param frame_index
 * @return
 */
int QNodeReceiver::getFrameHertz(const int &frame_index) {
	return frame_hertz[frame_index];
}

/**
 *
 * @param frame_index
 * @return
 */
QString QNodeReceiver::getFrameInactivity(const int &frame_index) {
	return frame_inactivity[frame_index];
}

/***********************************************
 SETTER
 ***********************************************/

/**
 *
 */
void QNodeReceiver::clearAllFrameAddresses() {
	frame_address_list.clear();
}

/**
 *
 * @param address
 */
void QNodeReceiver::addFrameAddress(QString address) {
	frame_address_list.append(address);
}

/**
 *
 * @param frame_index
 * @param address
 */
void QNodeReceiver::setFrameAddress(const int &frame_index, QString address) {
	frame_address_list.replace(frame_index, address);
	if (address != assign_address && address != ignore_address) {
		QString frame("Frame ");
		if (frame_index < 10) {
			frame.append(QString("0%1").arg(frame_index));
		} else {
			frame.append(QString::number(frame_index));
		}
		display(ACTIVE_FRAME, frame.append(QString(" >>> ")).append(address));
	}
}

/**
 *
 * @param frame_index
 * @param value
 */
void QNodeReceiver::setFrameHertz(const int &frame_index, const int &value) {
	frame_hertz[frame_index] = value;
}

/**
 *
 * @param frame_index
 * @param time
 */
void QNodeReceiver::setFrameInactivity(const int &frame_index, QString time) {
	frame_inactivity[frame_index] = time;
}

/**
 *
 * @param frame_index
 * @param time
 */
void QNodeReceiver::setLastUpdate(const int &frame_index,
		const ros::Time &time) {
	tf_last_message_stamp[frame_index] = time;
}

/**
 *
 * @param boolean
 */
void QNodeReceiver::setResetModelSignal(const bool &boolean) {
	reset_model_signal = boolean;
}

/**
 *
 * @param boolean
 */
void QNodeReceiver::setDisplayEulerSignal(const bool &boolean) {
	display_euler_signal = boolean;
}

/**
 *
 * @param frame_index
 */
void QNodeReceiver::setDisplayEulerFrame(const int &frame_index) {
	display_euler_frame = frame_index;
}

/**
 *
 * @return
 */
bool QNodeReceiver::socketReady() {

	if (!socketCreation()) {
		display(ERROR, QString("Socket creation failed!"));
		return false;
	}
	if (!socketOption()) {
		display(ERROR, QString("Socket option setting failed!"));
		return false;
	}
	if (!socketBinding()) {
		display(ERROR, QString("Socket binding failed!"));
		return false;
	}
	return true;
}

/**
 *
 * @return
 */
bool QNodeReceiver::socketCreation() {

	//create socket
	udp_socket = socket(AF_INET, SOCK_DGRAM, 0);

	int flags = fcntl(udp_socket, F_GETFL);
	flags |= O_NONBLOCK;
	fcntl(udp_socket, F_SETFL, flags);

	if (udp_socket < 0) {
		return false;
	}
	return true;
}

/**
 *
 * @return
 */
bool QNodeReceiver::socketOption() {

	//set socket option (allow other sockets to bind to this port)
	int optval = 1;
	udp_socket_option = setsockopt(udp_socket, SOL_SOCKET, SO_REUSEADDR,
			&optval, sizeof optval);
	if (udp_socket_option < 0) {
		return false;
	}
	return true;
}

/**
 *
 * @return
 */
bool QNodeReceiver::socketBinding() {

	//bind socket to port
	udp_socket_binding = bind(udp_socket, (struct sockaddr *) &socket_address,
			sizeof(socket_address));

	if (udp_socket_binding < 0) {
		return false;
	}
	return true;
}

/**
 *
 */
void QNodeReceiver::getTimeFromSec() {

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
void QNodeReceiver::displayEulerAverage() {

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
void QNodeReceiver::resetEuler() {

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
void QNodeReceiver::resetFrameRotation() {

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		tf_rotation[i] = tf::Quaternion(0, 0, 0, sqrt(1.0));
		tf_rotation[i].normalize();
	}
}

/**
 *
 */
void QNodeReceiver::resetFrameData() {

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		frame_hertz[i] = 0;
		frame_inactivity[i] = QTime(0, 0, 0).toString();
	}
}
