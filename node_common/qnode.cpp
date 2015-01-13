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
 *		Base class to setup a ROS node with basic methods
 *		and variables used by multiple specialized node classes
 *
 *	-------------------------------------------------------------
 * 	Authors:
 * 		Christian Benz 			<zneb_naitsirhc@web.de>
 * 		Christoph Döringer 		<christoph.doeringer@gmail.com>
 * 		Hendrik Pfeifer 		<hendrikpfeifer@gmail.com>
 * 		Heiko Reinemuth 		<heiko.reinemuth@gmail.com>
 ****************************************************************/

#include "qnode.hpp"

/*! \brief Constructor of QNode class
 *
 * @param argc Arguments
 * @param argv Arguments
 * @param name Node name
 */
QNode::QNode(int argc, char** argv, const std::string &name) :
		init_argc(argc), init_argv(argv), node_name(name) {

	//initialize urdf link names
	baseLinkName = "base_link_connector";
	frameLinkName[0] = "link_00_b_body";
	frameLinkName[1] = "link_01_b_head";
	frameLinkName[2] = "link_02_l_upper_arm";
	frameLinkName[3] = "link_03_l_lower_arm";
	frameLinkName[4] = "link_04_l_hand";
	frameLinkName[5] = "link_05_l_upper_leg";
	frameLinkName[6] = "link_06_l_lower_leg";
	frameLinkName[7] = "link_07_l_foot";
	frameLinkName[8] = "link_08_r_upper_arm";
	frameLinkName[9] = "link_09_r_lower_arm";
	frameLinkName[10] = "link_10_r_hand";
	frameLinkName[11] = "link_11_r_upper_leg";
	frameLinkName[12] = "link_12_r_lower_leg";
	frameLinkName[13] = "link_13_r_foot";

	//initialize urdf joint names
	baseJointName = "joint_base_connector";
	frameJointName[0] = "joint_00_b_core";
	frameJointName[1] = "joint_01_b_neck";
	frameJointName[2] = "joint_02_l_shoulder";
	frameJointName[3] = "joint_03_l_elbow";
	frameJointName[4] = "joint_04_l_wrist";
	frameJointName[5] = "joint_05_l_hip";
	frameJointName[6] = "joint_06_l_knee";
	frameJointName[7] = "joint_07_l_ankle";
	frameJointName[8] = "joint_08_r_shoulder";
	frameJointName[9] = "joint_09_r_elbow";
	frameJointName[10] = "joint_10_r_wrist";
	frameJointName[11] = "joint_11_r_hip";
	frameJointName[12] = "joint_12_r_knee";
	frameJointName[13] = "joint_13_r_ankle";
}

/*! \brief Destructor of QNode class
 *
 */
QNode::~QNode() {
	shutdownNode();
}

/***********************************************
 BASIC METHODS
 ***********************************************/

/*! \brief Initializes a ROS node
 *
 *		Connects with ROS master process,
 *		initializes a ROS node and starts it
 * @retval TRUE Initialization of ROS node successful
 * @retval FALSE Initialization of ROS node failed
 */
bool QNode::initNode() {

	//local IP4 host address
	std::string hostAddress = getIP4HostAddress();

	//URI of ROS master process
	std::string masterURI = "http://" + hostAddress + ":11311/";

	//remappings for ROS node initialization
	std::map<std::string, std::string> remappings;
	remappings["__hostname"] = hostAddress;
	remappings["__master"] = masterURI;

	//initialize ROS node
	ros::init(remappings, node_name);

	//display error message and return early if ROS master wasn't found
	if (!ros::master::check()) {
		display(ERROR,
				QString("Master ").append(masterURI.c_str()).append(
						QString(" not found")));
		return false;
	}

	//display info message if ROS master was found
	display(INFO,
			QString("Master ").append(masterURI.c_str()).append(
					QString(" found")));

	//start ROS node
	ros::start();

	return true;
}

/*! \brief Shuts a ROS node down
 *
 */
void QNode::shutdownNode() {

	//if ROS node is started
	if (ros::isStarted()) {

		//shut node down
		ros::shutdown();

		//wait for node to be shut down
		ros::waitForShutdown();
	}
	wait();
}

/*! \brief Returns QString representation of a frame index
 *
 * @param frame_index The index of a frame
 * @return QString representation of given frame index
 */
QString QNode::getFrameString(const int &frame_index) {

	if (frame_index < 10) {
		return QString("0%1").arg(frame_index);
	}
	return QString::number(frame_index);
}

/*! \brief Returns Display type of a frame index
 *
 * @param frame_index The index of a frame
 * @return Display type of given frame index
 */
DisplayType QNode::getFrameDisplayType(const int &frame_index) {

	DisplayType type;

	if (frame_index < 2) {
		type = FRAME01;
	} else if (frame_index < 5) {
		type = FRAME234;
	} else if (frame_index < 8) {
		type = FRAME567;
	} else if (frame_index < 11) {
		type = FRAME8910;
	} else {
		type = FRAME111213;
	}
	return type;
}

/*! \brief Appends a row to a listView model
 *
 *		Appends a new row with given style and message to a listView model
 * @param display_type Determines the style of the row
 * @param message The message to be displayed
 */
void QNode::display(const DisplayType &display_type, const QString &message) {

	//listView item to be added
	QStandardItem *listViewItem = new QStandardItem();

	//set font of listView item
	listViewItem->setData(QFont("Liberation Mono", 9, 75), Qt::FontRole);

	//listView item text
	QString text("");

	//set a certain style for a certain display type
	switch (display_type) {
	case (TIP): {
		text.append(QString("[TIP]: "));
		listViewItem->setData(QBrush(QColor(Qt::black)), Qt::ForegroundRole);
		break;
	}
	case (INFO): {
		text.append(QString("[INFO]: "));
		listViewItem->setData(QBrush(QColor(Qt::darkCyan)), Qt::ForegroundRole);
		break;
	}
	case (ERROR): {
		text.append(QString("[ERROR]: "));
		listViewItem->setData(QBrush(QColor(Qt::darkMagenta)),
				Qt::ForegroundRole);
		break;
	}
	case (EULER): {
		text.append(QString("[EULER ANGLES]: "));
		listViewItem->setData(QBrush(QColor(Qt::darkYellow)),
				Qt::ForegroundRole);
		break;
	}
	case (INACTIVE): {
		text.append(QString("[INACTIVE FRAMES]: "));
		listViewItem->setData(QBrush(QColor(Qt::darkYellow)),
				Qt::ForegroundRole);
		break;
	}
	case (ASYNCH): {
		text.append(QString("[ASYNCH]: "));
		listViewItem->setData(QBrush(QColor(Qt::darkYellow)),
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
	case (CALIBRATION): {
		text.append(QString("[CALIBRATION]: "));
		listViewItem->setData(QBrush(QColor(Qt::darkBlue)), Qt::ForegroundRole);
		break;
	}
		//should never be reached
	default: {
		break;
	}
	}

	//set text of listView item
	listViewItem->setText(text.append(message));

	//add listView item to listView model
	listViewModel.appendRow(listViewItem);

	//tell GUI to scroll down the listView
	Q_EMIT listViewModelUpdated();
}

/*! \brief Returns readable ROS timestamp for local time
 *
 *		Converts a ROS timestamp to local time and returns a readable string representation
 * @param stamp A given ROS timestamp
 * @return String representation of ROS time
 */
std::string QNode::rosTimeToLocalTime(ros::Time stamp) {

	//string representation of given timestamp
	std::string timeStr = to_iso_string(stamp.toBoost());

	//local system time
	time_t t = time(0);
	struct tm * now = localtime(&t);

	//convert hours integer variable to string
	std::stringstream ss;
	ss << now->tm_hour;
	std::string str = ss.str();

	//replace substring which holds hours with new hours string
	timeStr.replace(9, 2, str);

	return timeStr;
}

/***********************************************
 GETTER METHODS
 ***********************************************/

/*! \brief Getter for listView model
 * @return A listView model
 */
QStandardItemModel* QNode::getListViewModel() {
	return &listViewModel;
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
