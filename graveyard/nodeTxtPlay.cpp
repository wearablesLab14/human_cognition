/*
 * Authors:
 * 	Christian Benz,
 * 	Hendrik Pfeifer,
 * 	Heiko Reinemuth,
 * 	Christoph Döringer
 * Description:
 *	A node that reads data from a file
 *	and publishes the data with messages.
 */

#include <geometry_msgs/TransformStamped.h>
#include <fstream>

#include "header/parameter.h"
#include "header/print.h"
#include "header/system.h"

#define FREQ 100

typedef geometry_msgs::TransformStamped TransStmpMsg;

/**
 *
 *
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, "nodeTxtPlay");
	ros::NodeHandle nhPrv("~");
	ros::NodeHandle nhPup;

	//**********************PARAMETER START**********************

	//numbering for this node
	const int& numbering = getValue<int>(nhPrv, "numbering");

	//title of recording
	const std::string& title = getValue<std::string>(nhPup, "title");

	//topics
	const std::vector<std::string>& frameTopics = getArray<std::string>(nhPup,
			"frameTopics");

	//publication topic for this node
	const std::string& pubTopic = frameTopics.at(numbering);

	//record files
	const std::vector<std::string>& frameRecords = getArray<std::string>(nhPup,
			"frameRecords");

	//path to records folder
	std::string recordsTxtPath = getValue<std::string>(nhPup, "recordsTxtPath");

	//build record file path for this node
	const std::string& recFile = recordsTxtPath.append(title).append(
			frameRecords.at(numbering));

	//**********************PARAMETER END**********************

	//print warning and return early if record doesn't exist
	if (!fileExists(recFile)) {
		printStatus(ros::this_node::getName(), false, false);
		printStatusInfo(false, title);
		printStatusInfo(false, pubTopic);
		printStatusInfo(true, "THE TXT RECORD DOESN'T EXIST");
		return 0;
	}

	//create input stream
	std::ifstream recFileRead(recFile.c_str());

	//rate of loop
	ros::Rate loop_rate(FREQ);

	//publisher
	ros::Publisher pub = nhPup.advertise<geometry_msgs::TransformStamped>(
			pubTopic, 0);

	//TransformStamped message
	geometry_msgs::TransformStamped msg;

	//wait
	sleep(3.0);

	//while ROS is okay and end of record file is not reached
	while (ros::ok() && !recFileRead.eof()) {

		//assign record data to message
		recFileRead >> msg.header.frame_id >> msg.child_frame_id
				>> msg.header.stamp.sec >> msg.header.stamp.nsec
				>> msg.transform.rotation.x >> msg.transform.rotation.y
				>> msg.transform.rotation.z >> msg.transform.rotation.w
				>> msg.transform.translation.x >> msg.transform.translation.y
				>> msg.transform.translation.z;

		//publish message
		pub.publish(msg);

		//print status
		printStatus(ros::this_node::getName(), true, true);
		printStatusInfo(false, msg.header.stamp);
		printStatusInfo(false, title);
		printStatusInfo(true, pubTopic);

		loop_rate.sleep();
	}

	return 0;
}

