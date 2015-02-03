/****************************************************************
 *  Project:
 *  	Integrating Body- and Eye-Tracking to study Cognition in the Wild
 *	-------------------------------------------------------------
 * 	TU Darmstadt
 * 	Department Computer Science
 * 	Summer term 2014
 *	-------------------------------------------------------------
 *	File: qnode_receiver.cpp
 *	Description:
 *		Specialized node class to setup a ROS node for
 *		receiving UDP packets with rotation data, applying
 *		kinematic equations and publishing ROS tf messages
 *		to move a human model in ROS rviz
 *
 *	-------------------------------------------------------------
 * 	Authors:
 * 		Christian Benz 			<zneb_naitsirhc@web.de>
 * 		Christoph Döringer 		<christoph.doeringer@gmail.com>
 * 		Hendrik Pfeifer 		<hendrikpfeifer@gmail.com>
 * 		Heiko Reinemuth 		<heiko.reinemuth@gmail.com>
 ****************************************************************/
#include <string>
#include <sstream>
#include "qnode_receiver.hpp"

//tf::Quaternion offsetQuat[] = {};
tf::Quaternion* QNodeReceiver::offsetQuat = new tf::Quaternion[NUMBER_OF_FRAMES];
// correct the direction
int** offsetDirection = new int*[NUMBER_OF_FRAMES];

bool* QNodeReceiver::offsetCalculated = new bool[NUMBER_OF_FRAMES];
	
/*! \brief Constructor of QNodeReceiver class
 *
 *		Displays messages and initializes variables
 * @param argc Arguments
 * @param argv Arguments
 */
QNodeReceiver::QNodeReceiver(int argc, char** argv) :
		QNode(argc, argv, "receiver") {

	for (int i = 0; i < NUMBER_OF_FRAMES;i++)
	{
		// contains the achses-Order X Y Z
		offsetDirection[i] = new int[3];
	}
	
	display(TIP, "1. Connect to mocap");
	display(TIP, "2. Start ROS master process with 'roscore'");
	display(TIP,
			"Start RVIZ and load human model with 'roslaunch human_cognition rviz.launch'");
	display(TIP,
			"Record tf messages with 'roslaunch human_cognition record.launch title:=myTitle'");
	display(TIP, "Click 'reset' button in RVIZ");
	display(TIP,
			"Play tf messages with 'roslaunch human_cognition play.launch title:=myTitle'");

	/***********************************************************/
	//important to initialize with WHOLE PATH!!
	urdfFile =
			"/home/cognition/catkin_ws/src/human_cognition/devel/lib/human_cognition/human_real_size.urdf";
	
	//init for base transformation	
	frameYaw = 0;
	framePitch = 0;
	frameRoll = 0;

	//the calibration is not working initially
	doCalibration = false;
	// count !doCalibration
	countNotCalibration = 0;
	doOffsetCalc = false;
	
	//enum not mappable to string... grrr :-)
	limbStrings[0] = "BASE_BODY"; 
	limbStrings[1] = "BASE_HEAD"; 
	limbStrings[2] = "LEFT_UPPER_ARM"; 
	limbStrings[3] = "LEFT_LOWER_ARM"; 
	limbStrings[4] = "LEFT_HAND"; 
	limbStrings[5] = "LEFT_UPPER_LEG"; 
	limbStrings[6] = "LEFT_LOWER_LEG";
	limbStrings[7] = "LEFT_FOOT";
	limbStrings[8] = "RIGHT_UPPER_ARM";
	limbStrings[9] = "RIGHT_LOWER_ARM";
	limbStrings[10] = "RIGHT_HAND";
	limbStrings[11] = "RIGHT_UPPER_LEG";
	limbStrings[12] = "RIGHT_LOWER_LEG";
	limbStrings[13] = "RIGHT_FOOT";

	//initialize socket and address variables
	udpSocket = -1;
	udpSocketBinding = -1;
	udpSocketOption = -1;
	sensorAddressLength = 0;
	sensorPacketSize = 0;

	//set socket address to 0
	memset((char *) &socketAddress, 0, sizeof(socketAddress));

	//set sensor address to 0
	memset((char *) &sensorAddress, 0, sizeof(sensorAddress));

	//fill socket address
	socketAddress.sin_family = AF_INET;
	socketAddress.sin_addr.s_addr = htonl(INADDR_ANY);
	socketAddress.sin_port = htons(PORT);

	//sensor address length
	sensorAddressLength = sizeof(sensorAddress);

	/***********************************************************/

	//initialize address placeholders
	stdAssignAddress = QString(TO_ASSIGN_ADDRESS);
	stdIgnoreAddress = QString(TO_IGNORE_ADDRESS);

	/***********************************************************/

	//initialize settings variables
	signalPerformance = false;
	signalEdison = false;
	signalEuler = false;
	frameEuler = 0;
	signalInactivity = false;
	signalAsync = false;
	valueAsync = 0;
	signalResetModel = false;

	/***********************************************************/

	//initialize euler variables
	initEuler();

	//initialize frame variables
	initFrameData();
}

/***********************************************
 IMPLEMENTATION OF PURE VIRTUAL METHODS
 ***********************************************/

/*! \brief Initializes a receiver node
 *
 *		Initializes a ROS node, a socket for receiving
 *		and ROS tf messages
 * @retval TRUE Initialization successful
 * @retval FALSE Initialization failed
 */
bool QNodeReceiver::readyForAction() {

	//return early if initialization of ROS node failed
	if (!initNode()) {
		return false;
	}

	//return early if socket setup failed
	if (!socketReady()) {
		return false;
	}

	//initialize ROS tf messages
	initMessages();

	//display a ready to receive message if node is started
	if (ros::isStarted()) {
		display(INFO, QString("Ready to receive"));
	}
	return true;
}

/*! \brief Starts QThread
 *
 */
void QNodeReceiver::startAction() {
	start();
}

/*! \brief Shuts ROS node down and stops QThread
 *
 */
void QNodeReceiver::stopAction() {
	shutdownNode();
}

/*! \brief Run method of QThread
 *
 */
void QNodeReceiver::run() {

	//tf message publisher
	tf::TransformBroadcaster tfPublisher;

	/***********************************************************/

	//duration of initial publishing
	ros::Duration initialTimeout(2.0);

	//start time of initial publisher loop
	ros::Time initialStartTime = ros::Time::now();

	/***********************************************************/

	//publish initial tf data for a fixed duration
	while (ros::ok() && ((ros::Time::now() - initialStartTime) < initialTimeout)) {

		//update timestamp of base tf message
		tfBaseMsg.stamp_ = ros::Time::now();

		//publish base tf message
		tfPublisher.sendTransform(tfBaseMsg);

		/***********************************************************/

		//for all frames
		for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

			//update timestamp of frame tf message
			tfFrameMsg[i].stamp_ = ros::Time::now();

			//update/initialize  last stamp of frame with current stamp
			frameLastMsgStamp[i] = tfFrameMsg[i].stamp_;

			//publish frame tf message
			tfPublisher.sendTransform(tfFrameMsg[i]);
		}
	}

	/***********************************************************/

	//reset euler variables
	initEuler();

	//reset frame variables
	initFrameData();

	/***********************************************************/

	//current frame of sensor data packet
	int currFrame;

	//current address of sensor data packet
	QString currAddress;

	//frame y-axis rotation correction
	tf::Quaternion yRotCorr(0, sqrt(0.5), 0, -sqrt(0.5));
	yRotCorr.normalize();

	/***********************************************************/

	//duration of a defined interval for Hertz calculation and GUI updates
	ros::Duration oneSecInterval(1.0);

	//start time of receive loop(
	ros::Time receiveStartTime = ros::Time::now();

	/***********************************************************/

	display(INFO, QString("Receiving has started"));

	tf::Quaternion baseQuat = tf::Quaternion(0, 0, 0, 1);

	//receive sensor data packets while ROS is okay
	tf::Quaternion baseConnector;
	while (ros::ok()) {

		//receive bytes to fill struct
		sensorPacketSize = recvfrom(udpSocket, (char*) &sensorPacketData,
				sizeof(sensorPacketData), 0, (struct sockaddr *) &sensorAddress,
				&sensorAddressLength);

		//if bytes were received
		if (sensorPacketSize > 0) {
			/***********************************************************/

			//update current address with address of sensor data packet
			currAddress = QString(inet_ntoa(sensorAddress.sin_addr));
			
			
			// TODO Nghia, timeStampe 
			//receive bytes to fill struct
				
				
					// timeStampToString						
					std::ostringstream ss;					
					ss << sensorPacketData.timestamp;
					
					std::string  tmpCurrAddress =  currAddress.toUtf8().constData() ;
					tmpCurrAddress = tmpCurrAddress.replace(0,10,"") + "."  + ss.str();		
					currAddress.clear();
					currAddress.append(tmpCurrAddress.c_str());
						
						
					/*				
					//sensorPacketData.timestamp.str
					std::cout << "currAddressto  " << currAddress.toUtf8().constData() <<
					"sensorPacketData.stamp_  " << sensorPacketData.timestamp <<
					"tmpCurrAddress  " << tmpCurrAddress <<
					std::endl; 
					*/
				 
					 
					 
			
			//search address list for current address and return index if found
			currFrame = frameAddressList.indexOf(currAddress, 0);
			


			/***********************************************************/

			//if current address wasn't found
			if (currFrame < 0) {

				//search address list for an empty spot and return index if found
				currFrame = frameAddressList.indexOf(stdAssignAddress, 0);

				//if an empty spot was found
				if (currFrame >= 0) {

					//add current address to address list at current frame's position
					setFrameAddress(currFrame, currAddress);
				}
			}

			/***********************************************************/

			//if current frame is valid
			if (currFrame >= 0 && currFrame < NUMBER_OF_FRAMES) {

				//initialize rotation data (quaternion) with rotation data of sensor data packet
				tf::Quaternion quat(sensorPacketData.q1, sensorPacketData.q2,
						sensorPacketData.q3, sensorPacketData.q0);

				//normalize rotation data (quaternion)
				if (doOffsetCalc) {
					if (QNodeReceiver::offsetCalculated[currFrame]) {
						quat = quat * QNodeReceiver::offsetQuat[currFrame];
						quat.normalize();	
					} else {
						//the first iterations[currFrame] are used for the correction
						QNodeReceiver::offsetQuat[currFrame] = tf::Quaternion(
								QNodeReceiver::offsetQuat[currFrame].getX() + quat.getX(),
								QNodeReceiver::offsetQuat[currFrame].getY() + quat.getY(),
								QNodeReceiver::offsetQuat[currFrame].getZ() + quat.getZ(),
								QNodeReceiver::offsetQuat[currFrame].getW() + quat.getW());

						if (iterations[currFrame] == 100) {
							QNodeReceiver::offsetQuat[currFrame] = tf::Quaternion(
									QNodeReceiver::offsetQuat[currFrame].getX()
											/ iterations[currFrame],
									QNodeReceiver::offsetQuat[currFrame].getY()
											/ iterations[currFrame],
									QNodeReceiver::offsetQuat[currFrame].getZ()
											/ iterations[currFrame],
									QNodeReceiver::offsetQuat[currFrame].getW()
											/ iterations[currFrame]);							
							
							if(currFrame == 0)
								offsetHelper = offsetQuat[0];
							
							QNodeReceiver::offsetQuat[currFrame] = baseQuat * inverse(
									QNodeReceiver::offsetQuat[currFrame]);

							QNodeReceiver::offsetCalculated[currFrame] = true;
							display(INFO, QString("Offset calculated!"));
							
							
							//corrects the base body rotation to its original angle
							if (currFrame == 0) {
								tf::Matrix3x3(offsetHelper).getRPY(frameRoll, framePitch, frameYaw);
								offsetHelper.setRPY(0,0,-frameYaw);
								
								offsetQuat[currFrame]=
									offsetQuat[currFrame] *
									inverse(offsetHelper);
								}
							
						}
						iterations[currFrame]++;
					}
				} else {
					/***********************************************************/
					//correct original rotation for all frames except the feet frames
					//if (currFrame != 7 && currFrame != 13) {
					//multiply rotation data with rotation correction
					quat = quat * yRotCorr;

					//normalize rotation data
					quat.normalize();
					//}
				}

				//do calibration
				if (doCalibration) {
					if (firstTime) {
						display(CALIBRATION, 
							QString("Move your " + limbStrings[limbOrder[nextChangeIndex]]));

						firstTime = false;
					}
					
					//calculate roll, pitch and yaw from rotation data of sensor data packet
					int limbOrderindex = cmotionHelper::getIndex(limbOrder, currFrame,
							limbOrderLength);

					//display(CALIBRATION, QString::number(limbOrderindex));
					if (limbOrderindex >= 0) {
						
						// just continue to count, if no frame to calibration
						countNotCalibration = 0;
						
						// caculateCurrentChange to get the max chaged Coordinates
						diffCords[limbOrderindex] +=
								sqrt(
									pow((quat.getX()
											- lastStateCords[limbOrderindex][0]), 2)
									+ pow(
											(quat.getY()
													- lastStateCords[limbOrderindex][1]), 2)
									+ pow(
											(quat.getZ()
													- lastStateCords[limbOrderindex][2]), 2)
									+ pow(
											(quat.getW()
													- lastStateCords[limbOrderindex][3]), 2));


						// save the current coord/quat to last
						lastStateCords[limbOrderindex][0] = quat.getX();
						lastStateCords[limbOrderindex][1] = quat.getY();
						lastStateCords[limbOrderindex][2] = quat.getZ();
						lastStateCords[limbOrderindex][3] = quat.getW();

						
						//save if the new value > max value
						if (diffCords[limbOrderindex] > diffCords[maxIndex])
							maxIndex = limbOrderindex;
						
						if (diffCords[limbOrderindex] < diffCords[minIndex])
							minIndex = limbOrderindex;
						

						//if max value is bigger than 10, switch frames
						if (diffCords[limbOrderindex] > 3) {
							//which frames to switch
							framesToSwitch[0] = limbOrder[nextChangeIndex];
							framesToSwitch[1] = limbOrder[limbOrderindex];
							// limbOrder[limbOrderindex] == currFrame, müsste
							assert(limbOrder[limbOrderindex] == currFrame);
							
					
							std::cout << "framesToSwitch[0]: " << framesToSwitch[0]
									<< "   framesToSwitch[1]: " << framesToSwitch[1] << std::endl;
							//tell GUI to updated switch info
							Q_EMIT calibrationSwitchUpdated();
							
							/**** RESET VALUES ****/
							//limbOrder isn't used anymore
							diffCords[limbOrderindex] = 0;
							// -1 d.h. climb is deaktivated
							limbOrder[nextChangeIndex] = -1;
							//diffCords[14]=0;
							maxIndex = 14;
							nextChangeIndex++;
							firstTime = true;
							//set the actual limb to standard position!
					
							quat = baseQuat;
						}

						// slia
						if ((ros::Time::now() - receiveStartTime) >= oneSecInterval) {

							display(CALIBRATION,
									QString("MaxValue %1, Frame %2").arg(
											QString::number(diffCords[maxIndex], 'f', 3)).arg(
											QString::number(maxIndex, 'f', 1)));
						}

					}
					else
					{
							countNotCalibration++;
							if(countNotCalibration == 100)
							{
								doCalibration = false;
								display(CALIBRATION,QString("Calibration is done"));
							}
					}
				}

				tfFrameRot[currFrame] = tf::Quaternion(quat.getX(), quat.getY(),
							quat.getZ(), quat.getW());
							

				tfFrameRot[currFrame].normalize();
				tfOrigFrameRot[currFrame] = tfFrameRot[currFrame];

				/***********************************************************/

				//apply kinematic equation for selected frames
				if (currFrame != 0 && currFrame != 2 && currFrame != 5 && currFrame != 8
						&& currFrame != 11) {

					tfFrameRot[currFrame] = inverse(tfOrigFrameRot[currFrame - 1])
							* tfFrameRot[currFrame];

				} else if (currFrame != 0) {
					tfFrameRot[currFrame] = inverse(tfOrigFrameRot[0])
							* tfFrameRot[currFrame];
				}

						
				//normalize rotation data
				tfFrameRot[currFrame].normalize();

				/***********************************************************/


						
				//assign message of current frame with updated rotation data
				tfFrameMsg[currFrame].setRotation(tfFrameRot[currFrame]);
				
	
			

				/***********************************************************/

				//count number of frame updates
				frameUpdateCount[currFrame] = frameUpdateCount[currFrame] + 1;

				/***********************************************************/

				//if euler angles should be displayed and current frame is the desired frame
				if (signalEuler && currFrame == frameEuler) {

					//calculate roll, pitch and yaw from rotation data of sensor data packet
					tf::Matrix3x3(quat).getRPY(frameEulerY, frameEulerX, frameEulerZ);
					
				}

				/***********************************************************/

				//if human model should be resetted
				if (signalResetModel) {

					//reset rotations of all frames
					initFrameRotation();
					for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
						tfFrameMsg[i].setRotation(tfFrameRot[i]);
					}
					signalResetModel = false;
				}

				/***********************************************************/

				//loop over all frames
				for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

					// Nghia print Information to Terminal
					if ( i== 0 && (ros::Time::now() - receiveStartTime) >= oneSecInterval) 					
					{	
						double x,y,z;
						tf::Matrix3x3(tfFrameRot[0]).getRPY(x,y,z);
						
						std::cout 	//<< tfFrameRot[0].getX() << "   " 
									//<< tfFrameRot[0].getY() << "   " 
									//<< tfFrameRot[0].getZ() << "   \n" 
									//<< "X Offset " <<offsetQuat[0].getX() << "   " 
									//<< "Y Offset " <<offsetQuat[0].getY() << "   " 
									//<< "Z Offset " <<offsetQuat[0].getZ() << "   \n"
									<< "X angle " <<x / 3.14 * 180<< "   " 
									<< "Y angle " <<y / 3.14 * 180<< "   " 
									<< "Z angle " <<z / 3.14 * 180<< "  \n " 
									<< "currAddress: " << currAddress.toUtf8().constData() 
									<< " " << frameAddressList.at(currFrame).toUtf8().constData() 
									<< "   currFrame: " << currFrame 
									<< "   sensorPacketData.timestamp: " << sensorPacketData.timestamp 
									<< std::endl;
					}					
					
					//update timestamp of base tf message
					tfBaseMsg.stamp_ = ros::Time::now();

					//publish base tf message
					tfPublisher.sendTransform(tfBaseMsg);

					//update timestamp of frame tf message
					tfFrameMsg[i].stamp_ = ros::Time::now();

					//publish frame tf message
					tfPublisher.sendTransform(tfFrameMsg[i]);

					/***********************************************************/

					//if asynchronity should be displayed and frame is current frame
					if (!signalPerformance && signalAsync && i == currFrame) {

						//current stamp of frame
						ros::Time currStamp = tfFrameMsg[currFrame].stamp_;

						//last stamp of frame
						ros::Time lastStamp = frameLastMsgStamp[currFrame];

						//calculate time span in seconds between latest and last frame update
						frameAsynchSpanSec[currFrame] = currStamp.sec - lastStamp.sec;

						//calculate time span in nanoseconds between latest and last frame update
						if (currStamp.sec == lastStamp.sec) {
							frameAsynchSpanNsec[currFrame] = currStamp.nsec - lastStamp.nsec;
						} else {
							frameAsynchSpanNsec[currFrame] = currStamp.nsec
									+ (ONE_SEC_IN_NSEC - lastStamp.nsec);
						}

						//update last stamp of frame
						frameLastMsgStamp[currFrame] = tfFrameMsg[currFrame].stamp_;
					}
				}

				/***********************************************************/

				//if one second interval is reached
				if ((ros::Time::now() - receiveStartTime) >= oneSecInterval) {

					//if performance mode is unchecked
					if (!signalPerformance) {

						//if euler angles should be displayed
						if (signalEuler) {

							//display the latest euler angles for a desired frame
							displayFrameEuler();
						}

						//if frame inactivity should be displayed
						if (signalInactivity) {

							//display inactivity status for all frames
							displayFrameInactivity();
						}

						//if asynchronity should be displayed
						if (signalAsync) {

							//display asynchronity status for active frames
							displayFrameAsync();
						}
					}

					/***********************************************************/

					//loop over all frames
					for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

						//update hertz value to display
						frameHertzToDisplay[i] = frameUpdateCount[i];

						//reset update counter
						frameUpdateCount[i] = 0;
					}

					//tell GUI to updated frame info
					Q_EMIT frameDataUpdated();

					//set interval start time to now
					receiveStartTime = ros::Time::now();
				}
			}
		}
	}

	//display message if run method finishes
	display(INFO, QString("Receiving has stopped"));
}

/*! \brief changes the calculated offset of a and b
 *
 *
 */
void QNodeReceiver::switchOffset(const int &frame_index_a,
		const int &frame_index_b) {
	offsetHelper = QNodeReceiver::offsetQuat[frame_index_a];
	QNodeReceiver::offsetQuat[frame_index_a] =
			QNodeReceiver::offsetQuat[frame_index_b];
	QNodeReceiver::offsetQuat[frame_index_b] = offsetHelper;
	bool offsetHelper = QNodeReceiver::offsetCalculated[frame_index_a];
	QNodeReceiver::offsetCalculated[frame_index_a] =
			QNodeReceiver::offsetCalculated[frame_index_b];
	QNodeReceiver::offsetCalculated[frame_index_b] = offsetHelper;
	display(INFO, QString("Offset switched!"));
}

/*! \activates the calibration and resets its values
 *
 *
 */
void QNodeReceiver::activateCalibration() {
	display(CALIBRATION, QString("Calibration activated!"));
	//initialize calibration variables
	calibrationCounter = 1;
	limbOrderLength = 14;

	// which order the calibration is done
	limbOrder[0] = RIGHT_HAND;
	limbOrder[1] = RIGHT_LOWER_ARM;
	limbOrder[2] = RIGHT_UPPER_ARM;
	limbOrder[3] = LEFT_HAND;
	limbOrder[4] = LEFT_LOWER_ARM;
	limbOrder[5] = LEFT_UPPER_ARM;
	limbOrder[6] = RIGHT_FOOT;
	limbOrder[7] = RIGHT_LOWER_LEG;
	limbOrder[8] = RIGHT_UPPER_LEG;
	limbOrder[9] = LEFT_FOOT;
	limbOrder[10] = LEFT_LOWER_LEG;
	limbOrder[11] = LEFT_UPPER_LEG;
	limbOrder[12] = BASE_HEAD;
	limbOrder[13] = BASE_BODY;

	for (int i = 0; i <= limbOrderLength; i++)
		diffCords[i] = 0;

	//start calibration
	doCalibration = true;
	//the index with the maximum value initially is 14
	maxIndex = 14;
	//to initialize a text message
	firstTime = true;
	//next limborder to be changed, initially 0
	nextChangeIndex = 0;
}

/*! \calculates the offset and resets its values
 *
 *
 */
void QNodeReceiver::calculateOffset() {
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		//if(!offsetCalculated[currFrame])
		QNodeReceiver::offsetQuat[i] = tf::Quaternion(0, 0, 0, 0);
		iterations[i] = 0;
		QNodeReceiver::offsetCalculated[i] = false;
	}
	baseQuat = tf::Quaternion(0, 0, 0, 1);
	doOffsetCalc = true;
}

/***********************************************
 GETTER
 ***********************************************/

/*! \brief Returns standard assign placeholder address
 *
 * @return Standard assign placeholder address
 */
QString QNodeReceiver::getAssignAddress() {
	return stdAssignAddress;
}

/*! \brief Returns standard ignore placeholder address
 *
 * @return Standard ignore placeholder address
 */
QString QNodeReceiver::getIgnoreAddress() {
	return stdIgnoreAddress;
}

/*! \brief Returns a frames address for a given index
 *
 * @param frame_index The index of a frame
 * @return The address of the given frame
 */
QString QNodeReceiver::getFrameAddress(const int &frame_index) {
	return frameAddressList.at(frame_index);
}

/*! \brief Returns a frames hertz value to display for a given index
 *
 * @param frame_index The index of a frame
 * @return The hertz value to display of the given frame
 */
int QNodeReceiver::getFrameHertzToDisplay(const int &frame_index) {
	return frameHertzToDisplay[frame_index];
}

/*! \brief returns a array of two integer values to be switched
 *
 * @return the frames to be switched because of calibration
 */
int* QNodeReceiver::getFramesToSwitch() {
	return framesToSwitch;
}

/***********************************************
 SETTER
 ***********************************************/

/*! \brief Adds a address to the list of frame addresses
 *
 * @param address The address to be added
 */
void QNodeReceiver::addFrameAddress(QString address) {
	frameAddressList.append(address);
}

/*! \brief Sets address for a given frame
 *
 * @param frame_index The index of a frame
 * @param address The address of a frame
 */
void QNodeReceiver::setFrameAddress(const int &frame_index, QString address) {

	//replace address at the given frame index with the given address
	frameAddressList.replace(frame_index, address);

	//if the given address is not one of the placeholder addresses
	if (address != stdAssignAddress && address != stdIgnoreAddress) {

		//display frame message with given frame index and address
		QString frame(getFrameString(frame_index));
		frame.append(QString(" >>> "));
		frame.append(address);
		display(getFrameDisplayType(frame_index), frame);
	}
}

/*! \brief Sets reset model signal
 *
 * @param boolean Boolean value to be set
 */
void QNodeReceiver::setSignalResetModel(const bool &boolean) {
	signalResetModel = boolean;
}

/*! \brief Sets performance signal
 *
 * @param boolean Boolean value to be set
 */
void QNodeReceiver::setSignalPerformance(const bool &boolean) {
	signalEuler = boolean;
}

/*! \spam Edison - executed as thread
 *
 *
 */
void spamEdison() {
	while (true) {
		boost::this_thread::sleep(boost::posix_time::milliseconds(8));
		system("/bin/bash -c \"/bin/echo -n spam >/dev/udp/192.168.0.100/6060\"");
	}
}

/*! \brief Sets edison signal & starts spam thread
 *
 * @param boolean Boolean value to be set
 */
void QNodeReceiver::setSignalEdison(const bool &boolean) {
	signalEdison = boolean;
	if (signalEdison)
		boost::thread thread_Edison(spamEdison);
	//if(!signalEdison)
	//	thread_Edison.interrupt();
}

/*! \brief Sets euler signal
 *
 * @param boolean Boolean value to be set
 */
void QNodeReceiver::setSignalEuler(const bool &boolean) {
	signalEuler = boolean;
}

/*! \brief Sets euler frame to be displayed
 *
 * @param frame_index The index of a frame
 */
void QNodeReceiver::setFrameEuler(const int &frame_index) {
	frameEuler = frame_index;
}

/*! \brief Sets inactivity signal
 *
 * @param boolean Boolean value to be set
 */
void QNodeReceiver::setSignalInactivity(const bool &boolean) {
	signalInactivity = boolean;
}

/*! \brief Sets asynchronity signal
 *
 * @param boolean Boolean value to be set
 */
void QNodeReceiver::setSignalAsync(const bool &boolean) {
	signalAsync = boolean;
}

/*! \brief Sets asynchronity value
 *
 * @param value The minimum hertz value
 */
void QNodeReceiver::setValueAsync(const int &value) {
	valueAsync = value;
}

/***********************************************
 PRIVATE HELPER METHODS
 ***********************************************/

/*! \brief Initializes socket for receiving
 *
 * @retval TRUE Socket initialization was successful
 * @retval FALSE Socket initialization failed
 */
bool QNodeReceiver::socketReady() {

	//display error message and return early if socket creation failed
	if (!socketCreation()) {
		display(ERROR, QString("Socket creation failed!"));
		return false;
	}

	//display error message and return early if socket option setting failed
	if (!socketOption()) {
		display(ERROR, QString("Socket option setting failed!"));
		return false;
	}

	//display error message and return early if socket binding failed
	if (!socketBinding()) {
		display(ERROR, QString("Socket binding failed!"));
		return false;
	}
	return true;
}

/*! \brief Creates an UDP socket for receiving
 *
 * @retval TRUE Socket creation successful
 * @retval FALSE Socket creation failed
 */
bool QNodeReceiver::socketCreation() {

	//create socket
	udpSocket = socket(AF_INET, SOCK_DGRAM, 0);

	//return early if socket creation failed
	if (udpSocket < 0) {
		return false;
	}
	return true;
}

/*! \brief Sets UDP socket option
 *
 * @retval TRUE Socket option setting successful
 * @retval FALSE Socket option setting failed
 */
bool QNodeReceiver::socketOption() {

	//set socket option (allow other sockets to bind to this port)
	int optval = 1;
	udpSocketOption = setsockopt(udpSocket, SOL_SOCKET, SO_REUSEADDR, &optval,
			sizeof optval);

	//return early if socket option setting failed
	if (udpSocketOption < 0) {
		return false;
	}
	return true;
}

/*! \brief Bind UDP socket to port
 *
 * @retval TRUE Socket binding successful
 * @retval FALSE Socket binding failed
 */
bool QNodeReceiver::socketBinding() {

	//bind socket to port
	udpSocketBinding = bind(udpSocket, (struct sockaddr *) &socketAddress,
			sizeof(socketAddress));

	//return early if socket binding failed
	if (udpSocketBinding < 0) {
		return false;
	}
	return true;
}

/*! \brief Initialize tf messages to be published
 *
 */
void QNodeReceiver::initMessages() {

	//links for base message
	std::string baseParentLink;
	std::string baseChildLink;

	//relative joint origin for base message
	tf::Vector3 baseJointRelative;

	//links for frame messages
	std::string frameParentLink[NUMBER_OF_FRAMES];
	std::string frameChildLink[NUMBER_OF_FRAMES];

	//relative joint origin for frame messages
	tf::Vector3 frameJointRelative[NUMBER_OF_FRAMES];

	//read from urdf if parsing of urdf file is successful
	if (model.initFile(urdfFile)) {

		//display message
		display(INFO, QString("URDF parsing successful"));

		//get parent and child links from urdf model for base
		baseParentLink = model.getJoint(baseJointName)->parent_link_name;
		baseChildLink = model.getJoint(baseJointName)->child_link_name;

		//get relative joint origin from urdf model for base
		baseJointRelative.setX(
				model.getJoint(baseJointName)->parent_to_joint_origin_transform.position.x);
		baseJointRelative.setY(
				model.getJoint(baseJointName)->parent_to_joint_origin_transform.position.y);
		baseJointRelative.setZ(
				model.getJoint(baseJointName)->parent_to_joint_origin_transform.position.z);


		for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

			//get parent and child links from urdf model for frames
			frameParentLink[i] = model.getJoint(frameJointName[i])->parent_link_name;
			frameChildLink[i] = model.getJoint(frameJointName[i])->child_link_name;

			//get relative joint origin from urdf model for frames
			frameJointRelative[i].setX(
					model.getJoint(frameJointName[i])->parent_to_joint_origin_transform.position.x);
			frameJointRelative[i].setY(
					model.getJoint(frameJointName[i])->parent_to_joint_origin_transform.position.y);
			frameJointRelative[i].setZ(
					model.getJoint(frameJointName[i])->parent_to_joint_origin_transform.position.z);
		}
	}

	//use standard links and joints if urdf parsing failed
	else {

		//display message
		display(INFO,
				QString("URDF parsing failed - using standard links and joints"));

		//set parent and child for base
		baseParentLink = "base_link";
		baseChildLink = baseLinkName;

		//set relative joint origin for base
		baseJointRelative = tf::Vector3(0, 0, 1);

		//set parent and child for frames
		for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

			if (i == 0) {
				frameParentLink[i] = baseLinkName;
			} else if (i == 2) {
				frameParentLink[i] = frameLinkName[0];
			} else if (i == 5) {
				frameParentLink[i] = frameLinkName[0];
			} else if (i == 8) {
				frameParentLink[i] = frameLinkName[0];
			} else if (i == 11) {
				frameParentLink[i] = frameLinkName[0];
			} else {
				frameParentLink[i] = frameLinkName[i - 1];
			}
			frameChildLink[i] = frameLinkName[i];
		}

		//set relative joint origin for frames
		frameJointRelative[0] = tf::Vector3(0, 0, 0);
		frameJointRelative[1] = tf::Vector3(0, 0, 0.53);
		frameJointRelative[2] = tf::Vector3(0, 0.3, 0.52);
		frameJointRelative[3] = tf::Vector3(0, 0, -0.25);
		frameJointRelative[4] = tf::Vector3(0, 0, -0.27);
		frameJointRelative[5] = tf::Vector3(0, 0.1, 0);
		frameJointRelative[6] = tf::Vector3(0, 0, -0.55);
		frameJointRelative[7] = tf::Vector3(0, 0, -0.45);
		frameJointRelative[8] = tf::Vector3(0, -0.3, 0.52);
		frameJointRelative[9] = tf::Vector3(0, 0, -0.25);
		frameJointRelative[10] = tf::Vector3(0, 0, -0.27);
		frameJointRelative[11] = tf::Vector3(0, -0.1, 0);
		frameJointRelative[12] = tf::Vector3(0, 0, -0.55);
		frameJointRelative[13] = tf::Vector3(0, 0, -0.45);
	}

	//standard base rotation
	tfBaseRot = tf::Quaternion(0, 0, 0, sqrt(1.0));
	tfBaseRot.normalize();

	//init base tf message
	tfBaseMsg.frame_id_ = baseParentLink;
	tfBaseMsg.child_frame_id_ = baseChildLink;
	tfBaseMsg.setOrigin(baseJointRelative);
	// base root
	tfBaseMsg.setRotation(tfBaseRot);

	//init frame rotations
	initFrameRotation();

	//init frame tf messages
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

		tfFrameMsg[i].frame_id_ = frameParentLink[i];
		tfFrameMsg[i].child_frame_id_ = frameChildLink[i];
		tfFrameMsg[i].setOrigin(frameJointRelative[i]);
		tfFrameMsg[i].setRotation(tfFrameRot[i]);
	}
}

/*! \brief Initialize euler variables
 *
 */
void QNodeReceiver::initEuler() {

	frameEulerX = 0;
	frameEulerY = 0;
	frameEulerZ = 0;
}

/*! \brief Initialize frame rotation
 *
 */
void QNodeReceiver::initFrameRotation() {

	//loop over all frames
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

		//initial rotation for all frames
		tfFrameRot[i] = tf::Quaternion(0, 0, 0, sqrt(1.0));

		//normalize rotation
		tfFrameRot[i].normalize();
		tfOrigFrameRot[i] = tfFrameRot[i];
	}
}

/*! \brief Initialize frame variables
 *
 */
void QNodeReceiver::initFrameData() {

	//loop over all frames
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		frameUpdateCount[i] = 0;
		frameHertzToDisplay[i] = 0;
		frameAsynchSpanSec[i] = 0;
		frameAsynchSpanNsec[i] = 0;
	}
}

/*! \brief Display latest euler angles for the desired euler frame
 *
 */
void QNodeReceiver::displayFrameEuler() {
	display(EULER,
			QString("x-axis %1, y-axis %2, z-axis %3").arg(
					QString::number(frameEulerX, 'f', 3)).arg(
					QString::number(frameEulerY, 'f', 3)).arg(
					QString::number(frameEulerZ, 'f', 3)));
}

/*! \brief Display inactivity status for all frames
 *
 */
void QNodeReceiver::displayFrameInactivity() {

	//number of inactive frames
	int inactiveFramesCount = 0;

	//warning text to be displayed
	QString inactivityWarning("");

	//loop over all frames
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

		//if frame address isn't a ignore placeholder address
		//and the current hertz value is zero
		if (frameAddressList.at(i) != stdIgnoreAddress
				&& frameHertzToDisplay[i] == 0) {

			//increase counter of inactive frames
			inactiveFramesCount++;

			//add frame string to warning
			inactivityWarning.append(getFrameString(i));
			inactivityWarning.append(QString(" "));
		}
	}
	//if at least one frame is inactive
	if (inactiveFramesCount > 0) {
		display(INACTIVE, inactivityWarning);
		//if all selected frames are active
	} else {
		display(INFO, QString("All selected frames are active"));
	}
}

/*! \brief Display asynchronity status for all frames
 *
 */
void QNodeReceiver::displayFrameAsync() {

	//loop over all frames
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

		//if frame is active and current hertz value is under a fixed minimum limit
		if (frameHertzToDisplay[i] != 0 && frameHertzToDisplay[i] < valueAsync) {

			//warning text to be displayed
			QString asynchronityWarning(getFrameString(i));

			//add asynchronity info to warning text
			asynchronityWarning.append(QString(" < "));
			asynchronityWarning.append(QString::number(valueAsync));
			asynchronityWarning.append(QString("Hz (latest-last stamp: "));
			if (frameAsynchSpanSec[i] != 0) {
				asynchronityWarning.append(QString::number(frameAsynchSpanSec[i]));
				asynchronityWarning.append(QString(" sec. "));
			}
			asynchronityWarning.append(QString::number(frameAsynchSpanNsec[i]));
			asynchronityWarning.append(QString(" nsec.)"));

			display(ASYNCH, asynchronityWarning);
		}
	}
}

