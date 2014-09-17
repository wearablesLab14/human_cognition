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
	Q_EMIT listViewModelUpdated();
}

/***********************************************
 GETTER METHODS
 ***********************************************/

/**
 *
 * @return
 */
QStandardItemModel* QNode::getListViewModel() {
	return &list_view_model;
}

/***********************************************
 PRIVATE HELPER METHODS
 ***********************************************/

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

