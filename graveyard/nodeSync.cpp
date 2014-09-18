#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef geometry_msgs::TransformStamped TransStmpMsg;
typedef message_filters::Subscriber<TransStmpMsg> FilterSubscriber;
typedef message_filters::sync_policies::ApproximateTime<TransStmpMsg,
		TransStmpMsg, TransStmpMsg, TransStmpMsg> SyncPolicy;

class SyncAndPublish {

private:

	FilterSubscriber filterSub0;
	FilterSubscriber filterSub1;
	FilterSubscriber filterSub2;
	FilterSubscriber filterSub3;
	message_filters::Synchronizer<SyncPolicy> sync;
	boost::shared_ptr<const TransStmpMsg> *subMsgs;
	tf::TransformBroadcaster broadcaster;

public:

	SyncAndPublish(ros::NodeHandle nhPup) :
			filterSub0(nhPup, "t0", 0), filterSub1(nhPup, "t1", 0), filterSub2(
					nhPup, "t2", 0), filterSub3(nhPup, "t3", 0), sync(
					SyncPolicy(0), filterSub0, filterSub1, filterSub2,
					filterSub3) {

		subMsgs = new boost::shared_ptr<const TransStmpMsg>[4];

		sync.registerCallback(
				boost::bind(&SyncAndPublish::callback, this, _1, _2, _3, _4));
	}

	void callback(const TransStmpMsg::ConstPtr& msg0,
			const TransStmpMsg::ConstPtr& msg1,
			const TransStmpMsg::ConstPtr& msg2,
			const TransStmpMsg::ConstPtr& msg3) {

		subMsgs[0] = msg0;
		subMsgs[1] = msg1;
		subMsgs[2] = msg2;
		subMsgs[3] = msg3;

		tf::Quaternion q[4];

		for (unsigned int i = 0; i < 4; i++) {

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

int main(int argc, char **argv) {

	ros::init(argc, argv, "nodeSync");
	ros::NodeHandle nhPrv("~");
	ros::NodeHandle nhPup;

	SyncAndPublish subAndPub(nhPup);

	while (ros::ok()) {
		ros::spin();
	}

	return 0;
}
