#ifndef human_cognition_QNODE_RECEIVER_HPP_
#define human_cognition_QNODE_RECEIVER_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <boost/thread.hpp>
#include "../node_common/qnode.hpp"
#include "./cmotionHelper.h"
#endif

/*
 * define which frameIndex belongs to the binded limb
 */
enum LimbFrameOrder {
 BASE_BODY = 0,
 BASE_HEAD = 1,
 LEFT_UPPER_ARM = 2,
 LEFT_LOWER_ARM = 3,
 LEFT_HAND = 4,
 LEFT_UPPER_LEG = 5,
 LEFT_LOWER_LEG = 6,
 LEFT_FOOT = 7,
 RIGHT_UPPER_ARM = 8,
 RIGHT_LOWER_ARM = 9,
 RIGHT_HAND = 10,
 RIGHT_UPPER_LEG = 11,
 RIGHT_LOWER_LEG = 12,
 RIGHT_FOOT = 13
};

/*! \brief Specialized node class for receiving UDP packets
 * @author Christian Benz <zneb_naitsirhc@web.de>
 * @author Christoph Döringer <christoph.doeringer@gmail.com>
 * @author Hendrik Pfeifer <hendrikpfeifer@gmail.com>
 * @author Heiko Reinemuth <heiko.reinemuth@gmail.com>
 */
class QNodeReceiver: public QNode {

public:
	QNodeReceiver(int argc, char** argv);
	virtual ~QNodeReceiver() {
	}

	/***********************************************
	 IMPLEMENTATION OF PURE VIRTUAL METHODS
	 ***********************************************/
	bool readyForAction();
	void startAction();
	void stopAction();
	void run();
	virtual void switchOffset(const int &frame_index_a, const int &frame_index_b);
	virtual void activateCalibration(const bool check_boxes0,
									 const bool check_boxes1,
									 const bool check_boxes2,
									 const bool check_boxes3,
									 const bool check_boxes4,
									 const bool check_boxes5,
									 const bool check_boxes6,
									 const bool check_boxes7,
									 const bool check_boxes8,
									 const bool check_boxes9,
									 const bool check_boxes10,
									 const bool check_boxes11,
									 const bool check_boxes12,
									 const bool check_boxes13);
	virtual void calculateOffset();

	/***********************************************
	 GETTER
	 ***********************************************/
	virtual QString getAssignAddress();
	virtual QString getIgnoreAddress();
	virtual QString getFrameAddress(const int &frame_index);
	virtual int getFrameHertzToDisplay(const int &frame_index);
	virtual int* getFramesToSwitch();

	/***********************************************
	 SETTER
	 ***********************************************/
	virtual void addFrameAddress(QString address);
	virtual void setFrameAddress(const int &frame_index, QString address);
	virtual void setSignalResetModel(const bool &boolean);
	virtual void setSignalPerformance(const bool &boolean);
	virtual void setSignalEdison(const bool &boolean);
	virtual void setSignalEuler(const bool &boolean);
	virtual void setFrameEuler(const int &frame_index);
	virtual void setSignalInactivity(const bool &boolean);
	virtual void setSignalAsync(const bool &boolean);
	virtual void setValueAsync(const int &value);

	/***********************************************
	 STATIC
	 ***********************************************/
	static tf::Quaternion* offsetQuat ;
	static bool* offsetCalculated;

private:
	/***********************************************
	 PRIVATE HELPER METHODS
	 ***********************************************/
	bool socketReady();
	bool socketCreation();
	bool socketOption();
	bool socketBinding();
	void initEuler();
	void initMessages();
	void initFrameRotation();
	void initFrameData();
	void displayFrameEuler();
	void displayFrameInactivity();
	void displayFrameAsync();

	//socket and address variables
	int udpSocket;
	int udpSocketBinding;
	int udpSocketOption;
	struct sockaddr_in socketAddress;
	struct sockaddr_in sensorAddress;
	socklen_t sensorAddressLength;
	SensorData sensorPacketData;
	int sensorPacketSize;

	//address placeholders
	QString stdAssignAddress;
	QString stdIgnoreAddress;

	//settings variables
	bool signalPerformance;
	bool signalEdison;
	bool signalEuler;
	int frameEuler;
	bool signalInactivity;
	bool signalAsync;
	int valueAsync;
	bool signalResetModel;

	//euler angle variables
	double frameEulerX;
	double frameEulerY;
	double frameEulerZ;
	double frameYaw;
	double framePitch;
	double frameRoll;
	double frameYaw2;
	double framePitch2;
	double frameRoll2;

	//list of frame ip addresses
	QStringList frameAddressList;

	//current count of frame updates
	int frameUpdateCount[NUMBER_OF_FRAMES];

	//total count of frame updates in one second
	int frameHertzToDisplay[NUMBER_OF_FRAMES];

	//last frame message stamp
	ros::Time frameLastMsgStamp[NUMBER_OF_FRAMES];

	//time between the latest and the last frame message stamp in seconds
	int frameAsynchSpanSec[NUMBER_OF_FRAMES];

	//time between the latest and the last frame message stamp in nanoseconds
	int frameAsynchSpanNsec[NUMBER_OF_FRAMES];

	//urdf data model
	urdf::Model model;
	//init urdf File path
	std::string urdfFile;

	//base connector rotation
	tf::Quaternion tfBaseRot;

	//base connector message
	tf::StampedTransform tfBaseMsg;

	//frame rotation
	tf::Quaternion tfFrameRot[NUMBER_OF_FRAMES];
	tf::Quaternion tfOrigFrameRot[NUMBER_OF_FRAMES];

	//frame message
	tf::StampedTransform tfFrameMsg[NUMBER_OF_FRAMES];

	//offset
	bool doOffsetCalc;
	int iterations[14];
	tf::Quaternion baseQuat;
	tf::Quaternion baseQuatFoot;

	//counts frames for calibration calculation
	int calibrationCounter;
	//which frame is searched to calibrate
	// int calibrateLimb;
	//which frames to switch
	int framesToSwitch[2];
	bool doCalibration;
	bool firstTime;
	int countNotCalibration;

	int limbOrder[NUMBER_OF_FRAMES];
	int limbOrderLength;
	double diffCords[NUMBER_OF_FRAMES + 1];
	double lastStateCords[NUMBER_OF_FRAMES][4];
	int maxIndex;
	int minIndex;
	int nextChangeIndex;
	//base connector rotation
	tf::Quaternion offsetSwitchHelper;
	tf::Quaternion offsetYawBack;
	tf::Quaternion offsetLittleDevices;
	bool offsetYawBackCalculated[NUMBER_OF_FRAMES];
	//euler angle array for calibration
	double initEulerXYZ[NUMBER_OF_FRAMES][3];
	double eulerXYZ[NUMBER_OF_FRAMES][3];
	//void switchOffsets(int s, int t);
	QString limbStrings[14];

};

#endif
