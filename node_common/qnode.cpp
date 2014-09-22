/****************************************************************
 *  Project:
 *  	Integrating Body- and Eye-Tracking to study Cognition in the Wild
 *	-------------------------------------------------------------
 * 	TU Darmstadt
 * 	Department Computer Science
 * 	Summer term 2014
 *	-------------------------------------------------------------
 *	File: qnode.cpp
 *	Description:
 *
 *	-------------------------------------------------------------
 * 	Authors:
 * 		Christian Benz 			<zneb_naitsirhc@web.de>
 * 		Christoph Döringer 		<christoph.doeringer@gmail.com>
 * 		Hendrik Pfeifer 		<hendrikpfeifer@gmail.com>
 * 		Heiko Reinemuth 		<heiko.reinemuth@gmail.com>
 ****************************************************************/

#include "qnode.hpp"

/**
 *
 * @param argc
 * @param argv
 * @param name
 */
QNode::QNode(int argc, char** argv, const std::string &name) :
		init_argc(argc), init_argv(argv), node_name(name) {

	link_name[0] = "link_00_b_body";
	link_name[1] = "link_01_b_head";
	link_name[2] = "link_02_l_upper_arm";
	link_name[3] = "link_03_l_lower_arm";
	link_name[4] = "link_04_l_hand";
	link_name[5] = "link_05_l_upper_leg";
	link_name[6] = "link_06_l_lower_leg";
	link_name[7] = "link_07_l_foot";
	link_name[8] = "link_08_r_upper_arm";
	link_name[9] = "link_09_r_lower_arm";
	link_name[10] = "link_10_r_hand";
	link_name[11] = "link_11_r_upper_leg";
	link_name[12] = "link_12_r_lower_leg";
	link_name[13] = "link_13_r_foot";

	joint_base_name = "joint_base_connector";
	joint_name[0] = "joint_00_b_core";
	joint_name[1] = "joint_01_b_neck";
	joint_name[2] = "joint_02_l_shoulder";
	joint_name[3] = "joint_03_l_elbow";
	joint_name[4] = "joint_04_l_wrist";
	joint_name[5] = "joint_05_l_hip";
	joint_name[6] = "joint_06_l_knee";
	joint_name[7] = "joint_07_l_ankle";
	joint_name[8] = "joint_08_r_shoulder";
	joint_name[9] = "joint_09_r_elbow";
	joint_name[10] = "joint_10_r_wrist";
	joint_name[11] = "joint_11_r_hip";
	joint_name[12] = "joint_12_r_knee";
	joint_name[13] = "joint_13_r_ankle";
}

/**
 *
 */
QNode::~QNode() {
	shutdownNode();
}

/***********************************************
 BASIC METHODS
 ***********************************************/

/**
 *
 * @return
 */
bool QNode::initNode() {

	std::string host_ip = getIP4HostAddress();
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
	return true;
}

/**
 *
 */
void QNode::shutdownNode() {
	if (ros::isStarted()) {
		ros::shutdown();
		ros::waitForShutdown();
	}
	wait();
}

/**
 *
 * @param frame_index
 * @return
 */
QString QNode::getFrameString(const int &frame_index) {

	if (frame_index < 10) {
		return QString("0%1").arg(frame_index);
	}
	return QString::number(frame_index);
}

/**
 *
 * @param frame_index
 * @return
 */
DisplayType QNode::getFrameDisplayType(const int &frame_index) {

	DisplayType level;

	if (frame_index < 2) {
		level = FRAME01;
	} else if (frame_index < 5) {
		level = FRAME234;
	} else if (frame_index < 8) {
		level = FRAME567;
	} else if (frame_index < 11) {
		level = FRAME8910;
	} else {
		level = FRAME111213;
	}
	return level;
}

/*! \brief Appends a row to a listView model
 *
 *
 * @param display_type determines the style of the row
 * @param message
 */
void QNode::display(const DisplayType &display_type, const QString &message) {

	QStandardItem *listViewItem = new QStandardItem();
	listViewItem->setData(QFont("Liberation Mono", 11, 75), Qt::FontRole);
	QString text("");

	switch (display_type) {
	case (INSTRUCTION): {
		text.append(QString("[INSTRUCTION] "));
		listViewItem->setData(QBrush(QColor(Qt::black)), Qt::ForegroundRole);
		break;
	}
	case (INFO): {
		text.append(QString("[INFO] "));
		listViewItem->setData(QBrush(QColor(Qt::darkCyan)), Qt::ForegroundRole);
		break;
	}
	case (WARNING): {
		text.append(QString("[WARNING] "));
		listViewItem->setData(QBrush(QColor(Qt::darkYellow)),
				Qt::ForegroundRole);
		break;
	}
	case (ERROR): {
		text.append(QString("[ERROR] "));
		listViewItem->setData(QBrush(QColor(Qt::darkMagenta)),
				Qt::ForegroundRole);
		break;
	}
	case (FRAME01): {
		listViewItem->setData(QBrush(QColor(5, 14, 64)), Qt::ForegroundRole);
		break;
	}
	case (FRAME234): {
		listViewItem->setData(QBrush(QColor(56, 118, 29)), Qt::ForegroundRole);
		break;
	}
	case (FRAME567): {
		listViewItem->setData(QBrush(QColor(180, 95, 6)), Qt::ForegroundRole);
		break;
	}
	case (FRAME8910): {
		listViewItem->setData(QBrush(QColor(255, 0, 0)), Qt::ForegroundRole);
		break;
	}
	case (FRAME111213): {
		listViewItem->setData(QBrush(QColor(153, 0, 255)), Qt::ForegroundRole);
		break;
	}
	}
	listViewItem->setText(text.append(message));
	list_view_model.appendRow(listViewItem);
	Q_EMIT listViewModelUpdated();
}

/***********************************************
 GETTER METHODS
 ***********************************************/

/*! \brief Getter for listView model
 * @return A listView model
 */
QStandardItemModel* QNode::getListViewModel() {
	return &list_view_model;
}

/***********************************************
 PRIVATE HELPER METHODS
 ***********************************************/

/*! \brief Returns IP4 host address of local machine
 *
 * 		Loops over linked list of network interfaces
 * 		to find the IP4 host address of the local machine
 * @return Host address in textual form
 */
std::string QNode::getIP4HostAddress() {

	struct ifaddrs * ifAddrStruct = NULL;
	struct ifaddrs * ifa = NULL;

	//create linked list of network interfaces on the host machine
	getifaddrs(&ifAddrStruct);

	//pointer for internet addresses
	void * tmpAddrPtr = NULL;

	//internet address buffer
	char address[INET_ADDRSTRLEN];

	//loop over linked list of network interfaces
	for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {

		//skip interface without a network address
		if (!ifa->ifa_addr) {
			continue;
		}

		//check if network address of interface is IP4
		if (ifa->ifa_addr->sa_family == AF_INET) {

			//set pointer to internet address
			tmpAddrPtr = &((struct sockaddr_in *) ifa->ifa_addr)->sin_addr;

			//convert internet address from binary to textual form
			inet_ntop(AF_INET, tmpAddrPtr, address, INET_ADDRSTRLEN);

			//leave loop if interface is 'wlan0'
			if (ifa->ifa_name == "wlan0") {
				break;
			}
		}
	}

	//reclaim storage
	if (ifAddrStruct != NULL) {
		freeifaddrs(ifAddrStruct);
	}

	//return host address
	return address;
}

