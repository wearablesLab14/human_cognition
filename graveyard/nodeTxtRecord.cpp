/*
 * Authors:
 * 	Christian Benz,
 * 	Hendrik Pfeifer,
 * 	Heiko Reinemuth,
 * 	Christoph Döringer
 * Description:
 * 	A node that records messages from a given topic into a file
 */

#include <geometry_msgs/TransformStamped.h>
#include <boost/filesystem.hpp>
#include <fstream>

#include "header/parameter.h"
#include "header/print.h"
#include "header/system.h"

typedef geometry_msgs::TransformStamped TransStmpMsg;

/**
 *
 *
 */
class SubscribeAndRecord {

private:

	//title of the recording
	const std::string& title;

	//topic this node subscribes to
	const std::string& subTopic;

	//record folder path
	const std::string& recFolder;

	//record file path
	const std::string& recFile;

	//output stream
	std::ofstream recFileWrite;

	//to do something when first message arrives
	bool initialized;

	//subscriber of this class
	ros::Subscriber sub;

public:

	/**
	 *
	 */
	SubscribeAndRecord(ros::NodeHandle nhPup, std::string title,
			std::string subTopic, std::string recFolder, std::string recFile) :
			title(title), subTopic(subTopic), recFolder(recFolder), recFile(
					recFile) {

		//initialize variables
		initialized = false;

		//initialize subscriber and register callback function
		sub = nhPup.subscribe<TransStmpMsg>(subTopic, 0,
				&SubscribeAndRecord::callback, this);
	}

	/**
	 *
	 *
	 */
	void callback(const TransStmpMsg::ConstPtr& msg) {

		//create record folder and file when first message arrives
		if (!initialized) {

			//create directory for record file if it doesn't exist
			boost::filesystem::create_directories(recFolder.c_str());

			//create output stream
			recFileWrite.open(recFile.c_str());

			//set initialized to true
			initialized = true;
		}

		//write message data into record file
		recFileWrite << msg->header.frame_id << " " << msg->child_frame_id
				<< " " << msg->header.stamp.sec << " " << msg->header.stamp.nsec << " " << msg->transform.rotation.x << " "
				<< msg->transform.rotation.y << " " << msg->transform.rotation.z
				<< " " << msg->transform.rotation.w << " "
				<< msg->transform.translation.x << " "
				<< msg->transform.translation.y << " "
				<< msg->transform.translation.z << "\n";

		//print status
		printStatus(ros::this_node::getName(), true, true);
		printStatusInfo(false, msg->header.stamp);
		printStatusInfo(false, title);
		printStatusInfo(true, subTopic);
	}
};

/**
 *
 *
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, "nodeTxtRecord");
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

	//subscription topic for this node
	const std::string& subTopic = frameTopics.at(numbering);

	//record files
	const std::vector<std::string>& frameRecords = getArray<std::string>(nhPup,
			"frameRecords");

	//path to records folder
	std::string recordsTxtPath0 = getValue<std::string>(nhPup, "recordsTxtPath");

	//build record folder path for this node
	const std::string& recFolder = recordsTxtPath0.append(title);

	//path to records folder
	std::string recordsTxtPath1 = getValue<std::string>(nhPup, "recordsTxtPath");

	//build record file path for this node
	const std::string& recFile = recordsTxtPath1.append(title).append(
			frameRecords.at(numbering));

	//**********************PARAMETER END**********************

	//print warning and return early if title is already in use
	if (fileExists(recFile)) {
		printStatus(ros::this_node::getName(), false, false);
		printStatusInfo(false, title);
		printStatusInfo(false, subTopic);
		printStatusInfo(true, "THE TITLE IS ALREADY IN USE");
		return 0;
	}

	//create class for subscribing and recording
	SubscribeAndRecord subAndRec(nhPup, title, subTopic, recFolder, recFile);

	//wait
	sleep(3.0);

	//process callbacks while ROS is ok
	while (ros::ok()) {
		ros::spin();
	}

	return 0;
}
