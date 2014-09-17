/*
 * Authors:
 * 	Christian Benz,
 * 	Hendrik Pfeifer,
 * 	Heiko Reinemuth,
 * 	Christoph Döringer
 * Description:
 *	A node that subscribes to multiple topics,
 *	synchronizes messages and publishes messages
 */

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "header/parameter.h"
#include "header/print.h"

typedef geometry_msgs::TransformStamped TransStmpMsg;
typedef message_filters::Subscriber<TransStmpMsg> FilterSubscriber;
typedef message_filters::sync_policies::ApproximateTime<TransStmpMsg,
		TransStmpMsg, TransStmpMsg, TransStmpMsg> SyncPolicy;

/**
 *
 *
 */
class SyncAndPublish {

private:

	//**********************PARAMETER START**********************

	//subscription topics
	std::vector<std::string> subTopics;

	//**********************PARAMETER END**********************

	//subscribers
	FilterSubscriber filterSub0;
	FilterSubscriber filterSub1;
	FilterSubscriber filterSub2;
	FilterSubscriber filterSub3;

	//synchronizer
	message_filters::Synchronizer<SyncPolicy> sync;

	//subscription messages
	boost::shared_ptr<const TransStmpMsg> *subMsgs;

	//Transform broadcaster
	tf::TransformBroadcaster broadcaster;

public:

	/**
	 *
	 */
	SyncAndPublish(ros::NodeHandle nhPup, int qSizeSub, int qSizePub,
			int qSizeSyc, std::vector<std::string> subTopics) :
			subTopics(subTopics), filterSub0(nhPup, subTopics.at(0), qSizeSub), filterSub1(
					nhPup, subTopics.at(1), qSizeSub), filterSub2(nhPup,
					subTopics.at(2), qSizeSub), filterSub3(nhPup,
					subTopics.at(3), qSizeSub), sync(SyncPolicy(qSizeSyc),
					filterSub0, filterSub1, filterSub2, filterSub3) {

		//initialize array
		subMsgs = new boost::shared_ptr<const TransStmpMsg>[subTopics.size()];

		//register human callback function
		sync.registerCallback(
				boost::bind(&SyncAndPublish::callback, this, _1, _2, _3, _4));
	}

	/**
	 *
	 */
	void callback(const TransStmpMsg::ConstPtr& msg0,
			const TransStmpMsg::ConstPtr& msg1,
			const TransStmpMsg::ConstPtr& msg2,
			const TransStmpMsg::ConstPtr& msg3) {

		subMsgs[0] = msg0;
		subMsgs[1] = msg1;
		subMsgs[2] = msg2;
		subMsgs[3] = msg3;

		tf::Quaternion q[4];

		for (unsigned int i = 0; i < subTopics.size(); i++) {

			q[i] = tf::Quaternion(subMsgs[i]->transform.rotation.x,
					subMsgs[i]->transform.rotation.y,
					subMsgs[i]->transform.rotation.z,
					subMsgs[i]->transform.rotation.w);

			if (i == 1 || i == 2 || i == 3) {
				q[i] = inverse(q[i - 1]) * q[i];
			}

			broadcaster.sendTransform(
					tf::StampedTransform(
							tf::Transform(q[i],
									tf::Vector3(
											subMsgs[i]->transform.translation.x,
											subMsgs[i]->transform.translation.y,
											subMsgs[i]->transform.translation.z)),
							subMsgs[i]->header.stamp,
							subMsgs[i]->header.frame_id,
							subMsgs[i]->child_frame_id));
		}
	}
};

/**
 *
 *
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, "nodeSync");
	ros::NodeHandle nhPrv("~");
	ros::NodeHandle nhPup;

	//**********************PARAMETER START**********************

	//subscription topics for this node
	const std::vector<std::string>& subTopics = getArray<std::string>(nhPup,
			"left_sub_topics");

	//queue sizes for this node
	const int& qSizeSub = getValue<int>(nhPrv, "qSizeSub");
	const int& qSizePub = getValue<int>(nhPrv, "qSizePub");
	const int& qSizeSyc = getValue<int>(nhPrv, "qSizeSyc");

	//**********************PARAMETER END**********************

	//create class for synchronizing and publishing
	SyncAndPublish subAndPub(nhPup, qSizeSub, qSizePub, qSizeSyc, subTopics);

	//process callbacks while ROS is ok
	while (ros::ok()) {
		ros::spin();
	}

	return 0;
}
