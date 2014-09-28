/****************************************************************
 *  Project:
 *  	Integrating Body- and Eye-Tracking to study Cognition in the Wild
 *	-------------------------------------------------------------
 * 	TU Darmstadt
 * 	Department Computer Science
 * 	Summer term 2014
 *	-------------------------------------------------------------
 *	File: qnode_listener.cpp
 *	Description:
 *		Specialized node class to setup a ROS node for
 *		listening to ROS tf messages
 *
 *	-------------------------------------------------------------
 * 	Authors:
 * 		Christian Benz 			<zneb_naitsirhc@web.de>
 * 		Christoph Döringer 		<christoph.doeringer@gmail.com>
 * 		Hendrik Pfeifer 		<hendrikpfeifer@gmail.com>
 * 		Heiko Reinemuth 		<heiko.reinemuth@gmail.com>
 ****************************************************************/

#include "qnode_listener.hpp"

/*! \brief EchoListener class
 *
 */
class EchoListener {
public:
	//listener for tf messages
	tf::TransformListener tf;
	EchoListener() {
	}
	;
	~EchoListener() {
	}
	;

private:
};

/*! \brief Constructor of QNodeListener class
 *
 *		Displays messages and initializes variables
 * @param argc Arguments
 * @param argv Arguments
 */
QNodeListener::QNodeListener(int argc, char** argv) :
		QNode(argc, argv, "listener") {

	display(TIP, "1. Connect to mocap");
	display(TIP, "2. Setup and start receiver");

	/***********************************************************/

	//initialize settings variables
	signalCoordinates = false;
	frameCoordinates = 0;
	signalRecord = false;
	fileRecord = "coordinates.csv";
}

/***********************************************
 IMPLEMENTATION OF PURE VIRTUAL METHODS
 ***********************************************/

/*! \brief Initializes a listener node
 *
 *		Initializes a ROS node for listening *
 * @retval TRUE Initialization successful
 * @retval FALSE Initialization failed
 */
bool QNodeListener::readyForAction() {

	//return early if initialization of ROS node failed
	if (!initNode()) {
		return false;
	}

	//display a ready to listen message if node is started
	if (ros::isStarted()) {
		display(INFO, QString("Ready to listen"));
	}
	return true;
}

/*! \brief Starts QThread
 *
 */
void QNodeListener::startAction() {
	start();
}

/*! \brief Shuts ROS node down and stops QThread
 *
 */
void QNodeListener::stopAction() {
	shutdownNode();
}

/*! \brief Run method of QThread
 *
 */
void QNodeListener::run() {

	//EchoListener object
	EchoListener echoListener;

	/***********************************************************/

	//target frame into which to transform
	std::string targetFrame = "/base_link";

	//loop over all frames
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

		QString message("Frame ");
		message.append(getFrameString(i));

		//wait a short duration for frame's tf message
		if (echoListener.tf.waitForTransform(targetFrame, frameLinkName[i],
				ros::Time(0), ros::Duration(0.3))) {

			//display frame found message if frame's tf message was received
			display(getFrameDisplayType(i), message.append(QString(" found")));
		} else {

			//display frame not found message if frame's tf message wasn't received
			display(getFrameDisplayType(i),
					message.append(QString(" NOT found")));
		}
	}

	/***********************************************************/

	//nanoseconds of last received tf message stamp
	int nsecOfLastMsgStamp = 0;

	//output file stream to record coordinates
	std::ofstream fileCoordinates;

	/***********************************************************/

	//duration of one second for GUI updates
	ros::Duration oneSecInterval(1.0);

	//start time of listen loop
	ros::Time listenStartTime = ros::Time::now();

	/***********************************************************/

	//listen for tf messages while ROS is okay
	while (ros::ok()) {

		try {

			//loop over all frames
			for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

				//look up latest tf message for frame
				echoListener.tf.lookupTransform(targetFrame, frameLinkName[i],
						ros::Time(0), frameMsg[i]);
			}

			/***********************************************************/

			//update data if nanoseconds of latest tf message stamp are not
			//equal to nanoseconds of last received tf message stamp
			if (frameMsg[0].stamp_.nsec != nsecOfLastMsgStamp) {

				//update nanoseconds variable
				nsecOfLastMsgStamp = frameMsg[0].stamp_.nsec;

				//loop over all frames
				for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

					//update absolute joint coordinates for frame
					absoluteJointCoordinates[i] = frameMsg[i].getOrigin();
				}

				/***********************************************************/

				//if coordinates should be recorded
				if (signalRecord) {

					//open output file stream if it isn't open yet
					if (!fileCoordinates.is_open()) {
						fileCoordinates.open(fileRecord.c_str(),
								std::ios::out | std::ios::app);
					}

					//loop over all frames
					for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

						//record frame index, readable timestamp of tf message
						//and absolute joint coordinates for frame
						fileCoordinates << i << ";"
								<< rosTimeToLocalTime(frameMsg[i].stamp_) << ";"
								<< absoluteJointCoordinates[i].getX() << ";"
								<< absoluteJointCoordinates[i].getY() << ";"
								<< absoluteJointCoordinates[i].getZ() << "\n";
					}
				}
			}

			/***********************************************************/

			//if one second interval is reached
			if ((ros::Time::now() - listenStartTime) >= oneSecInterval) {

				//if coordinates should be displayed
				if (signalCoordinates) {

					//display the latest coordinates for a desired frame
					displayCoordinates();
				}

				//if coordinates should be recorded
				if (signalRecord) {

					//display status message
					display(INFO, QString("Recording absolute coordinates"));
				}

				//set interval start time to now
				listenStartTime = ros::Time::now();
			}

			/***********************************************************/

			//catch possible exceptions by calling lookupTransform
		} catch (tf::TransformException& ex) {

			//if one second interval is reached
			if ((ros::Time::now() - listenStartTime) >= oneSecInterval) {

				//display exception message
				display(ERROR, QString("TransformException"));

				//set interval start time to now
				listenStartTime = ros::Time::now();
			}
		}
	}

	//close output file stream if coordinates were recorded
	if (signalRecord) {
		fileCoordinates.close();
	}

	//display message if run method finishes
	display(INFO, QString("Listening has stopped"));
}

/***********************************************
 SETTER
 ***********************************************/

/*! \brief Sets display coordinates signal
 *
 * @param boolean Boolean value to be set
 */
void QNodeListener::setSignalCoordinates(const bool &boolean) {
	signalCoordinates = boolean;
}

/*! \brief Sets coordinates frame to be displayed
 *
 * @param frame_index The index of a frame
 */
void QNodeListener::setFrameCoordinates(const int &frame_index) {
	frameCoordinates = frame_index;
}

/*! \brief Sets record coordinates signal
 *
 * @param boolean Boolean value to be set
 */
void QNodeListener::setSignalRecord(const bool &boolean) {
	signalRecord = boolean;
}

/*! \brief Sets coordinates record file
 *
 * @param file The file to be created
 */
void QNodeListener::setFileRecord(std::string file) {
	fileRecord = file;
}

/*! \brief Display latest coordinates for the desired coordinates frame
 *
 */
void QNodeListener::displayCoordinates() {

	QString msg(frameMsg[frameCoordinates].child_frame_id_.c_str());
	msg.append(QString(" x: "));
	msg.append(
			QString::number(absoluteJointCoordinates[frameCoordinates].getX()));
	msg.append(QString(" y: "));
	msg.append(
			QString::number(absoluteJointCoordinates[frameCoordinates].getY()));
	msg.append(QString(" z: "));
	msg.append(
			QString::number(absoluteJointCoordinates[frameCoordinates].getZ()));

	display(getFrameDisplayType(frameCoordinates), msg);
}
