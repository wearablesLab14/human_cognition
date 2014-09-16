#include "qnode.hpp"

QNode::QNode(int argc, char** argv, const std::string &name ) :
	init_argc(argc),
	init_argv(argv),
	node_name(name)
	{

	ip_to_assign = QString(TO_ASSIGN_ADDRESS);
	ip_to_ignore = QString(TO_IGNORE_ADDRESS);


	reset_model_signal = false;

	display_euler_signal = false;
	display_euler_frame = 0;

	display_coordinates_signal = false;
	display_coordinates_frame = 0;

	record_coordinates_signal = false;
	record_coordinates_file = "coordinates.csv";


	initAddresses();

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

			frame_hertz[i] = 0;
			frame_inactivity[i] = QTime(0, 0, 0).toString();
		}

	}

QNode::~QNode() {
	rosShutdown();
}

void QNode::rosShutdown() {
	if (ros::isStarted()) {
		ros::shutdown();
		ros::waitForShutdown();
	}
	wait();
}

/**
 *
 * @return
 */
bool QNode::nodeReady() {

	std::string host_ip = findHostAddress();
	std::string master_uri = "http://" + host_ip + ":11311/";

	std::map<std::string, std::string> remappings;
	remappings["__hostname"] = host_ip;
	remappings["__master"] = master_uri;

	ros::init(remappings, node_name);

	if (!ros::master::check()) {
		display(ERROR,
				QString("Master ").append(master_uri.c_str()).append(
						QString(" not found")));
		return false;
	}
	display(INFO,
			QString("Master ").append(master_uri.c_str()).append(
					QString(" found")));

	ros::start();
	//ros::NodeHandle nh;
	return true;
}

/**
 *
 * @return
 */
std::string QNode::findHostAddress() {

	struct ifaddrs * ifAddrStruct = NULL;
	struct ifaddrs * ifa = NULL;
	void * tmpAddrPtr = NULL;
	getifaddrs(&ifAddrStruct);
	char addressBuffer[INET_ADDRSTRLEN];

	for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
		if (!ifa->ifa_addr) {
			continue;
		}
		if (ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
			// is a valid IP4 Address
			tmpAddrPtr = &((struct sockaddr_in *) ifa->ifa_addr)->sin_addr;

			inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);

			if (ifa->ifa_name == "wlan0") {
				break;
			}
		}
	}
	if (ifAddrStruct != NULL) {
		freeifaddrs(ifAddrStruct);
	}
	return addressBuffer;
}

/**
 *
 * @param level
 * @param info
 */
void QNode::display(const DisplayLevel &level, const QString &info) {

	QStandardItem *listViewItem = new QStandardItem();
	listViewItem->setData(QFont("Liberation Mono", 11, 75), Qt::FontRole);
	listViewItem->setText(info);

	switch (level) {
	case (INSTRUCTION): {
		listViewItem->setData(QBrush(QColor(Qt::darkBlue)), Qt::ForegroundRole);
		break;
	}
	case (INFO): {
		listViewItem->setData(QBrush(QColor(Qt::darkGreen)),
				Qt::ForegroundRole);
		break;
	}
	case (ERROR): {
		listViewItem->setData(QBrush(QColor(Qt::darkRed)), Qt::ForegroundRole);
		break;
	}
	case (ACTIVE_FRAME): {
		listViewItem->setData(QBrush(QColor(Qt::darkCyan)), Qt::ForegroundRole);
		break;
	}
	}
	list_view_model.appendRow(listViewItem);
	Q_EMIT listInfoUpdated();
}


/**
 *
 */
void QNode::initAddresses() {

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
}

/**
 *
 */
void QNode::initMessages() {

	tf_links[0] = "link_00_b_body";
	tf_links[1] = "link_01_b_head";
	tf_links[2] = "link_02_l_upper_arm";
	tf_links[3] = "link_03_l_lower_arm";
	tf_links[4] = "link_04_l_hand";
	tf_links[5] = "link_05_l_upper_leg";
	tf_links[6] = "link_06_l_lower_leg";
	tf_links[7] = "link_07_l_foot";
	tf_links[8] = "link_08_r_upper_arm";
	tf_links[9] = "link_09_r_lower_arm";
	tf_links[10] = "link_10_r_hand";
	tf_links[11] = "link_11_r_upper_leg";
	tf_links[12] = "link_12_r_lower_leg";
	tf_links[13] = "link_13_r_foot";

	tf_base_rotation = tf::Quaternion(0, 0, 0, sqrt(1.0));
	tf_base_rotation.normalize();
	tf_base_message.frame_id_ = "base_link";
	tf_base_message.child_frame_id_ = "base_link_connector";
	tf_base_message.setOrigin(tf::Vector3(0, 0, 0.8));
	tf_base_message.setRotation(tf_base_rotation);

	//URDF Joints
	//************************************
	tf_joint_origin[0] = tf::Vector3(0, 0, 0);
	tf_joint_origin[1] = tf::Vector3(0, 0, 0.53);
	tf_link_parent[0] = "base_link_connector";
	tf_link_child[0] = tf_links[0];
	tf_link_parent[1] = tf_links[0];
	tf_link_child[1] = tf_links[1];
	//************************************
	tf_joint_origin[2] = tf::Vector3(0, 0.3, 0.52);
	tf_joint_origin[3] = tf::Vector3(0, 0, -0.25);
	tf_joint_origin[4] = tf::Vector3(0, 0, -0.27);
	tf_link_parent[2] = tf_links[0];
	tf_link_child[2] = tf_links[2];
	tf_link_parent[3] = tf_links[2];
	tf_link_child[3] = tf_links[3];
	tf_link_parent[4] = tf_links[3];
	tf_link_child[4] = tf_links[4];
	//************************************
	tf_joint_origin[5] = tf::Vector3(0, 0.1, 0);
	tf_joint_origin[6] = tf::Vector3(0, 0, -0.37);
	tf_joint_origin[7] = tf::Vector3(0, 0, -0.43);
	tf_link_parent[5] = "base_link_connector";
	tf_link_child[5] = tf_links[5];
	tf_link_parent[6] = tf_links[5];
	tf_link_child[6] = tf_links[6];
	tf_link_parent[7] = tf_links[6];
	tf_link_child[7] = tf_links[7];
	//************************************
	tf_joint_origin[8] = tf::Vector3(0, -0.3, 0.52);
	tf_joint_origin[9] = tf::Vector3(0, 0, -0.25);
	tf_joint_origin[10] = tf::Vector3(0, 0, -0.27);
	tf_link_parent[8] = tf_links[0];
	tf_link_child[8] = tf_links[8];
	tf_link_parent[9] = tf_links[8];
	tf_link_child[9] = tf_links[9];
	tf_link_parent[10] = tf_links[9];
	tf_link_child[10] = tf_links[10];
	//************************************
	tf_joint_origin[11] = tf::Vector3(0, -0.1, 0);
	tf_joint_origin[12] = tf::Vector3(0, 0, -0.37);
	tf_joint_origin[13] = tf::Vector3(0, 0, -0.43);
	tf_link_parent[11] = "base_link_connector";
	tf_link_child[11] = tf_links[11];
	tf_link_parent[12] = tf_links[11];
	tf_link_child[12] = tf_links[12];
	tf_link_parent[13] = tf_links[12];
	tf_link_child[13] = tf_links[13];
	//************************************

	resetRotation();

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

		//initialize TF messages
		tf_message[i].frame_id_ = tf_link_parent[i];
		tf_message[i].child_frame_id_ = tf_link_child[i];
		tf_message[i].setOrigin(tf_joint_origin[i]);
		tf_message[i].setRotation(tf_rotation[i]);
	}
}

/**
 *
 */
void QNode::resetRotation() {

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		//initial rotation
		tf_rotation[i] = tf::Quaternion(0, 0, 0, sqrt(1.0));
		tf_rotation[i].normalize();
	}
}

//**************************************************************************************

/**
 *
 * @return
 */
bool QNode::receiveReady() {

	if (!nodeReady()) {
		return false;
	}
	if (!socketReady()) {
		return false;
	}
	if (ros::isStarted()) {
		initMessages();
		display(INFO, QString("Ready to receive"));
	}
	return true;
}

/**
 *
 * @return
 */
bool QNode::listenReady() {

	if (!nodeReady()) {
		return false;
	}
	if (ros::isStarted()) {
		initMessages();
		display(INFO, QString("Ready to listen"));
	}
	return true;
}

/**
 *
 * @return
 */
bool QNode::socketReady() {

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
bool QNode::socketCreation() {

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
bool QNode::socketOption() {

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
bool QNode::socketBinding() {

	//bind socket to port
	udp_socket_binding = bind(udp_socket, (struct sockaddr *) &socket_address,
			sizeof(socket_address));

	if (udp_socket_binding < 0) {
		return false;
	}
	return true;
}

/***********************************************
GETTER
***********************************************/

/**
 *
 * @return
 */
QStandardItemModel* QNode::getListViewModel() {
	return &list_view_model;
}

/**
 *
 * @return
 */
QString QNode::getAssignAddress() {
	return ip_to_assign;
}

/**
 *
 * @return
 */
QString QNode::getIgnoreAddress() {
	return ip_to_ignore;
}

/**
 *
 * @param frame_index
 * @return
 */
QString QNode::getFrameAddress(const int &frame_index) {
	return frame_ip_list.at(frame_index);
}

/**
 *
 * @param frame_index
 * @return
 */
int QNode::getFrameHertz(const int &frame_index) {
	return frame_hertz[frame_index];
}

/**
 *
 * @param frame_index
 * @return
 */
QString QNode::getFrameInactivity(const int &frame_index) {
	return frame_inactivity[frame_index];
}

/***********************************************
SETTER
***********************************************/

/**
 *
 */
void QNode::clearAllFrameAddesses() {
	frame_ip_list.clear();
}

/**
 *
 * @param address
 */
void QNode::addFrameAddress(QString address) {
	frame_ip_list.append(address);
}

/**
 *
 * @param frame_index
 * @param address
 */
void QNode::setFrameAddress(const int &frame_index, QString address) {
	frame_ip_list.replace(frame_index, address);
	if (address != ip_to_assign && address != ip_to_ignore) {
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
void QNode::setFrameHertz(const int &frame_index, const int &value) {
	frame_hertz[frame_index] = value;
}

/**
 *
 * @param frame_index
 * @param time
 */
void QNode::setFrameInactivity(const int &frame_index, QString time) {
	frame_inactivity[frame_index] = time;
}

/**
 *
 * @param frame_index
 * @param time
 */
void QNode::setLastUpdate(const int &frame_index, const ros::Time &time) {
	tf_last_message_stamp[frame_index] = time;
}

/**
 *
 * @param boolean
 */
void QNode::setResetModelSignal(const bool &boolean) {
	reset_model_signal = boolean;
}

/**
 *
 * @param boolean
 */
void QNode::setDisplayEulerSignal(const bool &boolean) {
	display_euler_signal = boolean;
}

/**
 *
 * @param frame_index
 */
void QNode::setDisplayEulerFrame(const int &frame_index) {
	display_euler_frame = frame_index;
}

/**
 *
 * @param boolean
 */
void QNode::setDisplayCoordinatesSignal(const bool &boolean) {
	display_coordinates_signal = boolean;
}

/**
 *
 * @param frame_index
 */
void QNode::setDisplayCoordinatesFrame(const int &frame_index) {
	display_coordinates_frame = frame_index;
}

/**
 *
 * @param boolean
 */
void QNode::setRecordCoordinatesSignal(const bool &boolean) {
	record_coordinates_signal = boolean;
}

/**
 *
 * @param file
 */
void QNode::setRecordCoordinatesFile(std::string file) {
	record_coordinates_file = file;
}
