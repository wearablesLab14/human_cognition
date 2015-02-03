/****************************************************************
 *  Project:
 *  	Integrating Body- and Eye-Tracking to study Cognition in the Wild
 *	-------------------------------------------------------------
 * 	TU Darmstadt
 * 	Department Computer Science
 * 	Summer term 2014
 *	-------------------------------------------------------------
 *	File: window_receiver.cpp
 *	Description:
 *		GUI class for receiver node
 *
 *	-------------------------------------------------------------
 * 	Authors:
 * 		Christian Benz 			<zneb_naitsirhc@web.de>
 * 		Christoph Döringer 		<christoph.doeringer@gmail.com>
 * 		Hendrik Pfeifer 		<hendrikpfeifer@gmail.com>
 * 		Heiko Reinemuth 		<heiko.reinemuth@gmail.com>
 ****************************************************************/

#include "window_receiver.hpp"

using namespace Qt;

/*! \brief Constructor of WindowReceiver class
 *
 * @param node The node class for this GUI class
 * @param parent Parent for this window
 */
WindowReceiver::WindowReceiver(QNodeReceiver *node, QWidget *parent) :
		QMainWindow(parent), qnodeRecv(node) {

	//setup user interface
	uiRecv.setupUi(this);

	//frame checkBoxes for frame selection
	frameSelectCheckBox[0] = uiRecv.checkBox_00;
	frameSelectCheckBox[1] = uiRecv.checkBox_01;
	frameSelectCheckBox[2] = uiRecv.checkBox_02;
	frameSelectCheckBox[3] = uiRecv.checkBox_03;
	frameSelectCheckBox[4] = uiRecv.checkBox_04;
	frameSelectCheckBox[5] = uiRecv.checkBox_05;
	frameSelectCheckBox[6] = uiRecv.checkBox_06;
	frameSelectCheckBox[7] = uiRecv.checkBox_07;
	frameSelectCheckBox[8] = uiRecv.checkBox_08;
	frameSelectCheckBox[9] = uiRecv.checkBox_09;
	frameSelectCheckBox[10] = uiRecv.checkBox_10;
	frameSelectCheckBox[11] = uiRecv.checkBox_11;
	frameSelectCheckBox[12] = uiRecv.checkBox_12;
	frameSelectCheckBox[13] = uiRecv.checkBox_13;

	//frame comboBoxes for switch frame selection
	frameSwitchComboBox[0] = uiRecv.comboBox_00;
	frameSwitchComboBox[1] = uiRecv.comboBox_01;
	frameSwitchComboBox[2] = uiRecv.comboBox_02;
	frameSwitchComboBox[3] = uiRecv.comboBox_03;
	frameSwitchComboBox[4] = uiRecv.comboBox_04;
	frameSwitchComboBox[5] = uiRecv.comboBox_05;
	frameSwitchComboBox[6] = uiRecv.comboBox_06;
	frameSwitchComboBox[7] = uiRecv.comboBox_07;
	frameSwitchComboBox[8] = uiRecv.comboBox_08;
	frameSwitchComboBox[9] = uiRecv.comboBox_09;
	frameSwitchComboBox[10] = uiRecv.comboBox_10;
	frameSwitchComboBox[11] = uiRecv.comboBox_11;
	frameSwitchComboBox[12] = uiRecv.comboBox_12;
	frameSwitchComboBox[13] = uiRecv.comboBox_13;

	//frame lineEdits for hertz address info
	frameAddressLineEdit[0] = uiRecv.lineEditIP_00;
	frameAddressLineEdit[1] = uiRecv.lineEditIP_01;
	frameAddressLineEdit[2] = uiRecv.lineEditIP_02;
	frameAddressLineEdit[3] = uiRecv.lineEditIP_03;
	frameAddressLineEdit[4] = uiRecv.lineEditIP_04;
	frameAddressLineEdit[5] = uiRecv.lineEditIP_05;
	frameAddressLineEdit[6] = uiRecv.lineEditIP_06;
	frameAddressLineEdit[7] = uiRecv.lineEditIP_07;
	frameAddressLineEdit[8] = uiRecv.lineEditIP_08;
	frameAddressLineEdit[9] = uiRecv.lineEditIP_09;
	frameAddressLineEdit[10] = uiRecv.lineEditIP_10;
	frameAddressLineEdit[11] = uiRecv.lineEditIP_11;
	frameAddressLineEdit[12] = uiRecv.lineEditIP_12;
	frameAddressLineEdit[13] = uiRecv.lineEditIP_13;

	//frame lineEdits for hertz info
	frameHertzLineEdit[0] = uiRecv.lineEditHz_00;
	frameHertzLineEdit[1] = uiRecv.lineEditHz_01;
	frameHertzLineEdit[2] = uiRecv.lineEditHz_02;
	frameHertzLineEdit[3] = uiRecv.lineEditHz_03;
	frameHertzLineEdit[4] = uiRecv.lineEditHz_04;
	frameHertzLineEdit[5] = uiRecv.lineEditHz_05;
	frameHertzLineEdit[6] = uiRecv.lineEditHz_06;
	frameHertzLineEdit[7] = uiRecv.lineEditHz_07;
	frameHertzLineEdit[8] = uiRecv.lineEditHz_08;
	frameHertzLineEdit[9] = uiRecv.lineEditHz_09;
	frameHertzLineEdit[10] = uiRecv.lineEditHz_10;
	frameHertzLineEdit[11] = uiRecv.lineEditHz_11;
	frameHertzLineEdit[12] = uiRecv.lineEditHz_12;
	frameHertzLineEdit[13] = uiRecv.lineEditHz_13;

	//restore GUI state, values and lists from QSettings
	readSettings();

	//set listView model
	uiRecv.listViewInfo->setModel(qnodeRecv->getListViewModel());

	//connect signal calls of node with GUI update methods
	QObject::connect(qnodeRecv, SIGNAL(listViewModelUpdated()), this,
			SLOT(updateListView()));
	QObject::connect(qnodeRecv, SIGNAL(frameDataUpdated()), this,
			SLOT(updateFrameViews()));
	QObject::connect(qnodeRecv, SIGNAL(calibrationSwitchUpdated()), this,
			SLOT(updateCalibrationSwitch()));
}

/*! \brief Destructor of WindowReceiver class
 *
 */
WindowReceiver::~WindowReceiver() {
}

/*! \brief Saves GUI state, values and lists on window close event
 *
 * @param event Window closing event
 */
void WindowReceiver::closeEvent(QCloseEvent *event) {

	//save GUI state, values and lists in QSettings
	writeSettings();

	//close window
	QMainWindow::closeEvent(event);
}

/***********************************************
 GUI ACTION METHODS
 ***********************************************/

/*! \brief Initializes node for receiving, enables and disables GUI elements
 *
 */
void WindowReceiver::on_pushButtonReceiverSetup_clicked() {

	//initialize receiver node and socket for receiving
	if (qnodeRecv->readyForAction()) {

		//disable following GUI elements
		uiRecv.pushButtonReceiverSetup->setEnabled(false);

		//enable following GUI elements
		uiRecv.pushButtonReceiverStart->setEnabled(true);
		uiRecv.checkBoxPerformance->setEnabled(true);
		uiRecv.checkBoxEdison->setEnabled(true);
		uiRecv.pushButtonSwitch->setEnabled(true);
		uiRecv.pushButtonResetModel->setEnabled(true);
		uiRecv.pushButtonResetFrames->setEnabled(true);
		uiRecv.pushButtonCalibration->setEnabled(true);
		uiRecv.pushButtonOffset->setEnabled(true);

		//enable following GUI elements only when performance is unchecked
		if (!uiRecv.checkBoxPerformance->isChecked()) {
			uiRecv.checkBoxEuler->setEnabled(true);
			uiRecv.comboBoxEuler->setEnabled(true);
			uiRecv.checkBoxInactivity->setEnabled(true);
			uiRecv.checkBoxAsync->setEnabled(true);
			uiRecv.spinBoxAsync->setEnabled(true);
		}
	}
}

/*! \brief Starts thread for receiving, enables and disables GUI elements
 *
 */
void WindowReceiver::on_pushButtonReceiverStart_clicked() {

	//set node data
	qnodeRecv->setSignalPerformance(uiRecv.checkBoxPerformance->isChecked());
	qnodeRecv->setSignalEdison(uiRecv.checkBoxEdison->isChecked());
	qnodeRecv->setSignalEuler(uiRecv.checkBoxEuler->isChecked());
	qnodeRecv->setFrameEuler(uiRecv.comboBoxEuler->currentIndex());
	qnodeRecv->setSignalInactivity(uiRecv.checkBoxInactivity->isChecked());
	qnodeRecv->setSignalAsync(uiRecv.checkBoxAsync->isChecked());
	qnodeRecv->setValueAsync(uiRecv.spinBoxAsync->value());


	//start node thread
	qnodeRecv->startAction();

	//disable following GUI elements
	uiRecv.pushButtonReceiverStart->setEnabled(false);
	uiRecv.checkBoxPerformance->setEnabled(false);
	uiRecv.checkBoxEdison->setEnabled(false);
	uiRecv.checkBoxEuler->setEnabled(false);
	uiRecv.comboBoxEuler->setEnabled(false);
	uiRecv.checkBoxInactivity->setEnabled(false);
	uiRecv.checkBoxAsync->setEnabled(false);
	uiRecv.spinBoxAsync->setEnabled(false);

	//enable following GUI elements
	uiRecv.pushButtonReceiverStop->setEnabled(true);
}

/*! \brief Stops thread for receiving, enables and disables GUI elements
 *
 */
void WindowReceiver::on_pushButtonReceiverStop_clicked() {

	//stop node thread
	qnodeRecv->stopAction();

	//disable following GUI elements
	uiRecv.pushButtonReceiverStop->setEnabled(false);
	uiRecv.pushButtonSwitch->setEnabled(false);
	uiRecv.pushButtonResetModel->setEnabled(false);
	uiRecv.pushButtonResetFrames->setEnabled(false);
	uiRecv.pushButtonCalibration->setEnabled(false);
	uiRecv.pushButtonOffset->setEnabled(false);

	//enable following GUI elements
	uiRecv.pushButtonReceiverSetup->setEnabled(true);
}

/*! \brief Enables and disables certain GUI elements depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBoxPerformance_stateChanged(int state) {

	//performance modus is checked
	if (state == 2) {

		//disable following GUI elements
		uiRecv.checkBoxEuler->setEnabled(false);
		uiRecv.comboBoxEuler->setEnabled(false);
		uiRecv.checkBoxInactivity->setEnabled(false);
		uiRecv.checkBoxAsync->setEnabled(false);
		uiRecv.spinBoxAsync->setEnabled(false);

		//uncheck euler checkBox
		uiRecv.checkBoxEuler->setChecked(false);
		uiRecv.checkBoxInactivity->setChecked(false);
		uiRecv.checkBoxAsync->setChecked(false);

		//performance modus is unchecked
	} else if (state == 0) {

		//enable following GUI elements
		uiRecv.checkBoxEuler->setEnabled(true);
		uiRecv.checkBoxInactivity->setEnabled(true);
		uiRecv.checkBoxAsync->setEnabled(true);

		//enable euler comboBox only if euler checkBox is checked
		if (uiRecv.checkBoxEuler->isChecked()) {
			uiRecv.comboBoxEuler->setEnabled(true);
		}
		//enable async spinBox only if async checkBox is checked
		if (uiRecv.checkBoxAsync->isChecked()) {
			uiRecv.spinBoxAsync->setEnabled(true);
		}
	}
}

/*! \brief Enables and disables the euler comboBox depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBoxEdison_stateChanged(int state) {
	qnodeRecv->setSignalEdison(uiRecv.checkBoxEdison->isChecked());
}

/*! \brief Enables and disables the euler comboBox depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBoxEuler_stateChanged(int state) {

	//if euler angles is checked
	if (state == 2) {

		//enable following GUI element
		uiRecv.comboBoxEuler->setEnabled(true);

		//if euler angles is unchecked
	} else if (state == 0) {

		//disable following GUI element
		uiRecv.comboBoxEuler->setEnabled(false);
	}
}

/*! \brief Enables and disables the async spinBox depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBoxAsync_stateChanged(int state) {

	//if async is checked
	if (state == 2) {

		//enable following GUI element
		uiRecv.spinBoxAsync->setEnabled(true);

		//if async is unchecked
	} else if (state == 0) {

		//disable following GUI element
		uiRecv.spinBoxAsync->setEnabled(false);
	}
}

/*! \brief Opens the 'About' dialog window
 *
 */
void WindowReceiver::on_pushButtonAbout_clicked() {
	dialogAbout.setModal(true);
	dialogAbout.exec();
}

/*! \brief Opens the 'Help' dialog window
 *
 */
void WindowReceiver::on_pushButtonHelp_clicked() {
	dialogHelp.setModal(true);
	dialogHelp.exec();
}

/*! \brief Opens the 'Frames' dialog window
 *
 */
void WindowReceiver::on_pushButtonFrames_clicked() {
	dialogFrames.setModal(true);
	dialogFrames.exec();
}

/*! \brief Switch all frames with their selected switch frames
 *
 */
void WindowReceiver::on_pushButtonSwitch_clicked() {

	//loop over all frames
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
 		manual_frame_switch(i, frameSwitchComboBox[i]->currentIndex());
	
 	/*	
		//switch frames if a frame has not his own index as its selected switch frame
		if (i != frameSwitchComboBox[i]->currentIndex()) {
			frameSwitch(i, frameSwitchComboBox[i]->currentIndex());

			//reset current index of frame's switch comboBox
			frameSwitchComboBox[i]->setCurrentIndex(i);
		}
	*/
	}
}

/*! \change two frames
 *
 *
 */
void WindowReceiver::manual_frame_switch(int source, int target) {

		//switch frames if a frame has not his own index as its selected switch frame
		if (source != target) {
			frameSwitch(source, target);

			//reset current index of frame's switch comboBox
			frameSwitchComboBox[source]->setCurrentIndex(source);
		}
}

/*! \brief Changes frame selection and address depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBox_00_stateChanged(int state) {
	frameSelectChanged(0, state);
}

/*! \brief Changes frame selection and address depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBox_01_stateChanged(int state) {
	frameSelectChanged(1, state);
}

/*! \brief Changes frame selection and address depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBox_02_stateChanged(int state) {
	frameSelectChanged(2, state);
}

/*! \brief Changes frame selection and address depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBox_03_stateChanged(int state) {
	frameSelectChanged(3, state);
}

/*! \brief Changes frame selection and address depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBox_04_stateChanged(int state) {
	frameSelectChanged(4, state);
}

/*! \brief Changes frame selection and address depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBox_05_stateChanged(int state) {
	frameSelectChanged(5, state);
}

/*! \brief Changes frame selection and address depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBox_06_stateChanged(int state) {
	frameSelectChanged(6, state);
}

/*! \brief Changes frame selection and address depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBox_07_stateChanged(int state) {
	frameSelectChanged(7, state);
}

/*! \brief Changes frame selection and address depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBox_08_stateChanged(int state) {
	frameSelectChanged(8, state);
}

/*! \brief Changes frame selection and address depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBox_09_stateChanged(int state) {
	frameSelectChanged(9, state);
}

/*! \brief Changes frame selection and address depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBox_10_stateChanged(int state) {
	frameSelectChanged(10, state);
}

/*! \brief Changes frame selection and address depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBox_11_stateChanged(int state) {
	frameSelectChanged(11, state);
}

/*! \brief Changes frame selection and address depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBox_12_stateChanged(int state) {
	frameSelectChanged(12, state);
}

/*! \brief Changes frame selection and address depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowReceiver::on_checkBox_13_stateChanged(int state) {
	frameSelectChanged(13, state);
}

/*! \brief Resets rotation data for all frames to reset the model
 *
 *		Opens a QMessageBox to confirm resetting and
 *		sets the reset model signal if the action is confirmed
 */
void WindowReceiver::on_pushButtonResetModel_clicked() {

	//open message box to confirm resetting of model
	QMessageBox msgBox;
	msgBox.setWindowTitle(QString("Confirm reset"));
	msgBox.setText(QString("Are you sure you want to reset the model?"));
	msgBox.setInformativeText(
			QString(
					"Please be aware that resetting the model affects a possible recording!"));
	msgBox.setStandardButtons(QMessageBox::Apply | QMessageBox::Cancel);
	int msgBoxRet = msgBox.exec();

	//switch over message box return value
	switch (msgBoxRet) {
	//Apply was clicked
	case QMessageBox::Apply: {

		//set reset model signal
		qnodeRecv->setSignalResetModel(true);
		break;
	}
		//Cancel was clicked
	case QMessageBox::Cancel: {
		break;
	}
		//should never be reached
	default: {
		break;
	}
	}
}

/*! \brief Resets selection, switch frame and address for all frames
 *
 *		Opens a QMessageBox to confirm resetting and resets
 *		selection, switch frame and address if the action is confirmed
 */
void WindowReceiver::on_pushButtonResetFrames_clicked() {

	//open message box to confirm resetting of frames
	QMessageBox msgBox;
	msgBox.setWindowTitle(QString("Confirm reset"));
	msgBox.setText(QString("Are you sure you want to reset all frames?"));
	msgBox.setInformativeText(
			QString(
					"Please be aware that resetting all frames affects a possible recording!"));
	msgBox.setStandardButtons(QMessageBox::Apply | QMessageBox::Cancel);
	int msgBoxRet = msgBox.exec();

	//switch over message box return value
	switch (msgBoxRet) {
	//Apply was clicked
	case QMessageBox::Apply: {

		//loop over all frames
		for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

			//set frame address to standard placeholder assign address if frame is selected
			if (frameSelectCheckBox[i]->isChecked()) {
				qnodeRecv->setFrameAddress(i, qnodeRecv->getAssignAddress());
			}
			//mark frame as selected
			frameSelectCheckBox[i]->setChecked(true);

			//set switch frame index to frame's own index
			frameSwitchComboBox[i]->setCurrentIndex(i);
		}

		//update frame views
		updateFrameViews();
		break;
	}
		//Cancel was clicked
	case QMessageBox::Cancel: {
		break;
	}
		//should never be reached
	default: {
		break;
	}
	}
}

/*! \brief activates Calibration
 *
 *
 *
 */
void WindowReceiver::on_pushButtonCalibration_clicked() {
	bool boxes[NUMBER_OF_FRAMES];
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		if (frameSelectCheckBox[i]->isChecked()) {
			boxes[i] = true;
		} else {
			boxes[i] = false;
		}
	}
	//Activates the Calibration Function and resets all its initial values
	qnodeRecv->activateCalibration(	boxes[0],
									boxes[1], 
									boxes[2], 
									boxes[3], 
									boxes[4], 
									boxes[5], 
									boxes[6], 
									boxes[7], 
									boxes[8], 
									boxes[9], 
									boxes[10], 
									boxes[11], 
									boxes[12], 
									boxes[13]);
}

/*! \brief calculates the Offset
 *
 *
 *
 */
void WindowReceiver::on_pushButtonOffset_clicked() {

	//Activates the Offset Calculation and resets all its initial values
	qnodeRecv->calculateOffset();

}

/***********************************************
 MODEL SIGNAL METHODS
 ***********************************************/

/*! \brief Scrolls listView to the bottom
 *
 */
void WindowReceiver::updateListView() {

	uiRecv.listViewInfo->scrollToBottom();
}

/*! \brief Updates all frame views
 *
 */
void WindowReceiver::updateFrameViews() {

	//loop over all frames
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

		//update frame address
		updateFrameAddressView(i);

		//update frame hertz
		updateFrameHertzView(i);
	}
}

/*! \brief switch items for calibration
 *
 */
void WindowReceiver::updateCalibrationSwitch() {
	int *framesToBeSwitched = qnodeRecv->getFramesToSwitch();
	manual_frame_switch(framesToBeSwitched[0], framesToBeSwitched[1]);
}

/***********************************************
 PRIVATE GUI METHODS
 ***********************************************/

/*! \brief Switches frame selection and address
 *
 * @param frame_index_a Frame A to be switched with frame B
 * @param frame_index_b Frame B to be switched with frame A
 */
void WindowReceiver::frameSwitch(const int &frame_index_a,
		const int &frame_index_b) {

	//save both frame addresses
	QString address_a = qnodeRecv->getFrameAddress(frame_index_a);
	QString address_b = qnodeRecv->getFrameAddress(frame_index_b);

	//save both frame selections
	bool boolean_a = frameSelectCheckBox[frame_index_a]->isChecked();
	bool boolean_b = frameSelectCheckBox[frame_index_b]->isChecked();

	//switch frame addresses
	qnodeRecv->setFrameAddress(frame_index_a, address_b);
	qnodeRecv->setFrameAddress(frame_index_b, address_a);

	//switch frame selections
	frameSelectCheckBox[frame_index_a]->setChecked(boolean_b);
	frameSelectCheckBox[frame_index_b]->setChecked(boolean_a);

	//switch calculated offsets
	qnodeRecv->switchOffset(frame_index_a, frame_index_b);

	//update all frame views
	updateFrameViews();
}

/*! \brief Changes frame address depending on the changed selection state
 *
 * @param frame_index The index of the frame
 * @param state The new state of the checkBox
 */
void WindowReceiver::frameSelectChanged(const int &frame_index, int state) {

	//set frame address to placeholder assign address if frame is now selected
	//and current frame address is the placeholder ignore address
	if (state == 2
			&& qnodeRecv->getFrameAddress(frame_index)
					== qnodeRecv->getIgnoreAddress()) {
		qnodeRecv->setFrameAddress(frame_index, qnodeRecv->getAssignAddress());

		//set frame address to placeholder ignore address if frame is not selected anymore
	} else if (state == 0) {
		qnodeRecv->setFrameAddress(frame_index, qnodeRecv->getIgnoreAddress());
	}

	//update frame address view
	updateFrameAddressView(frame_index);
}

/*! \brief Update frame address view
 *
 * @param frame_index The index of the frame
 */
void WindowReceiver::updateFrameAddressView(const int &frame_index) {
	frameAddressLineEdit[frame_index]->setText(
			qnodeRecv->getFrameAddress(frame_index));
}

/*! \brief Update frame hertz view
 *
 * @param frame_index The index of the frame
 */
void WindowReceiver::updateFrameHertzView(const int &frame_index) {
	frameHertzLineEdit[frame_index]->setText(
			QString::number(qnodeRecv->getFrameHertzToDisplay(frame_index)));
}

/*! \brief Restores GUI state, values and lists from QSettings
 *
 */
void WindowReceiver::readSettings() {

	//receiver settings
	QSettings settings(QString("TU Darmstadt"),
			QString("human_cognition_receiver"));

	//restore geometry and window state
	restoreGeometry(settings.value(QString("key_geometry")).toByteArray());
	restoreState(settings.value(QString("key_window_state")).toByteArray());

	//initialize standard frames list for comboBoxes
	QStringList comboBoxFrames;
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		comboBoxFrames.append(qnodeRecv->getFrameString(i));
	}

	//restore settings values
	uiRecv.checkBoxPerformance->setChecked(
			settings.value(QString("key_signal_performance"), true).toBool());
	uiRecv.checkBoxEdison->setChecked(
			settings.value(QString("key_signal_edison"), true).toBool());
	uiRecv.checkBoxEuler->setChecked(
			settings.value(QString("key_signal_euler"), false).toBool());
	uiRecv.comboBoxEuler->addItems(comboBoxFrames);
	uiRecv.comboBoxEuler->setCurrentIndex(
			settings.value(QString("key_frame_euler"), 0).toInt());
	uiRecv.checkBoxInactivity->setChecked(
			settings.value(QString("key_signal_inactivity"), false).toBool());
	uiRecv.checkBoxAsync->setChecked(
			settings.value(QString("key_signal_async"), false).toBool());
	uiRecv.spinBoxAsync->setValue(
			settings.value(QString("key_value_async"), 30).toInt());

	//start reading address list
	int addressListSize = settings.beginReadArray(QString("key_address_list"));

	//restore address list if list size is correct
	if (addressListSize == NUMBER_OF_FRAMES) {
		for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
			settings.setArrayIndex(i);
			qnodeRecv->addFrameAddress(
					settings.value(QString("key_address"),
							qnodeRecv->getAssignAddress()).toString());
		}
	}

	//stop reading address list
	settings.endArray();

	//initialize address list if list size isn't correct
	if (addressListSize != NUMBER_OF_FRAMES) {
		for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
			qnodeRecv->addFrameAddress(qnodeRecv->getAssignAddress());
		}
	}

	//set values of GUI elements
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		if (qnodeRecv->getFrameAddress(i) == qnodeRecv->getIgnoreAddress()) {
			frameSelectCheckBox[i]->setChecked(false);
		} else {
			frameSelectCheckBox[i]->setChecked(true);
		}
		frameSwitchComboBox[i]->addItems(comboBoxFrames);
		frameSwitchComboBox[i]->setCurrentIndex(i);
		updateFrameAddressView(i);
		updateFrameHertzView(i);
	}

	//disable following GUI elements
	uiRecv.checkBoxPerformance->setEnabled(false);
	uiRecv.checkBoxEdison->setEnabled(false);
	uiRecv.checkBoxEuler->setEnabled(false);
	uiRecv.comboBoxEuler->setEnabled(false);
	uiRecv.checkBoxInactivity->setEnabled(false);
	uiRecv.checkBoxAsync->setEnabled(false);
	uiRecv.spinBoxAsync->setEnabled(false);
	uiRecv.pushButtonReceiverStart->setEnabled(false);
	uiRecv.pushButtonReceiverStop->setEnabled(false);
	uiRecv.pushButtonSwitch->setEnabled(false);
	uiRecv.pushButtonResetModel->setEnabled(false);
	uiRecv.pushButtonResetFrames->setEnabled(false);
	uiRecv.pushButtonCalibration->setEnabled(false);
	uiRecv.pushButtonOffset->setEnabled(false);
}

/*! \brief Saves GUI state, values and lists in QSettings
 *
 */
void WindowReceiver::writeSettings() {

	//receiver settings
	QSettings settings(QString("TU Darmstadt"),
			QString("human_cognition_receiver"));

	//save geometry and window state
	settings.setValue(QString("key_geometry"), saveGeometry());
	settings.setValue(QString("key_window_state"), saveState());

	//save settings values
	settings.setValue(QString("key_signal_performance"),
			uiRecv.checkBoxPerformance->isChecked());
	settings.setValue(QString("key_signal_edison"),
			uiRecv.checkBoxEdison->isChecked());
	settings.setValue(QString("key_signal_euler"),
			uiRecv.checkBoxEuler->isChecked());
	settings.setValue(QString("key_frame_euler"),
			uiRecv.comboBoxEuler->currentIndex());
	settings.setValue(QString("key_signal_inactivity"),
			uiRecv.checkBoxInactivity->isChecked());
	settings.setValue(QString("key_signal_async"),
			uiRecv.checkBoxAsync->isChecked());
	settings.setValue(QString("key_value_async"), uiRecv.spinBoxAsync->value());

	//start writing address list
	settings.beginWriteArray(QString("key_address_list"));

	//write address list into settings
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		settings.setArrayIndex(i);
		settings.setValue(QString("key_address"),
				qnodeRecv->getFrameAddress(i));
	}

	//stop writing address list
	settings.endArray();
}
