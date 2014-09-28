#include "window_receiver.hpp"

using namespace Qt;

/**
 *
 * @param node
 * @param parent
 */
WindowReceiver::WindowReceiver(QNodeReceiver *node, QWidget *parent) :
		QMainWindow(parent), qnode_recv(node) {

	ui_recv.setupUi(this);
	frameSelectCheckBox[0] = ui_recv.checkBox_00;
	frameSelectCheckBox[1] = ui_recv.checkBox_01;
	frameSelectCheckBox[2] = ui_recv.checkBox_02;
	frameSelectCheckBox[3] = ui_recv.checkBox_03;
	frameSelectCheckBox[4] = ui_recv.checkBox_04;
	frameSelectCheckBox[5] = ui_recv.checkBox_05;
	frameSelectCheckBox[6] = ui_recv.checkBox_06;
	frameSelectCheckBox[7] = ui_recv.checkBox_07;
	frameSelectCheckBox[8] = ui_recv.checkBox_08;
	frameSelectCheckBox[9] = ui_recv.checkBox_09;
	frameSelectCheckBox[10] = ui_recv.checkBox_10;
	frameSelectCheckBox[11] = ui_recv.checkBox_11;
	frameSelectCheckBox[12] = ui_recv.checkBox_12;
	frameSelectCheckBox[13] = ui_recv.checkBox_13;

	frameSwitchComboBox[0] = ui_recv.comboBox_00;
	frameSwitchComboBox[1] = ui_recv.comboBox_01;
	frameSwitchComboBox[2] = ui_recv.comboBox_02;
	frameSwitchComboBox[3] = ui_recv.comboBox_03;
	frameSwitchComboBox[4] = ui_recv.comboBox_04;
	frameSwitchComboBox[5] = ui_recv.comboBox_05;
	frameSwitchComboBox[6] = ui_recv.comboBox_06;
	frameSwitchComboBox[7] = ui_recv.comboBox_07;
	frameSwitchComboBox[8] = ui_recv.comboBox_08;
	frameSwitchComboBox[9] = ui_recv.comboBox_09;
	frameSwitchComboBox[10] = ui_recv.comboBox_10;
	frameSwitchComboBox[11] = ui_recv.comboBox_11;
	frameSwitchComboBox[12] = ui_recv.comboBox_12;
	frameSwitchComboBox[13] = ui_recv.comboBox_13;

	frameAddressLineEdit[0] = ui_recv.lineEditIP_00;
	frameAddressLineEdit[1] = ui_recv.lineEditIP_01;
	frameAddressLineEdit[2] = ui_recv.lineEditIP_02;
	frameAddressLineEdit[3] = ui_recv.lineEditIP_03;
	frameAddressLineEdit[4] = ui_recv.lineEditIP_04;
	frameAddressLineEdit[5] = ui_recv.lineEditIP_05;
	frameAddressLineEdit[6] = ui_recv.lineEditIP_06;
	frameAddressLineEdit[7] = ui_recv.lineEditIP_07;
	frameAddressLineEdit[8] = ui_recv.lineEditIP_08;
	frameAddressLineEdit[9] = ui_recv.lineEditIP_09;
	frameAddressLineEdit[10] = ui_recv.lineEditIP_10;
	frameAddressLineEdit[11] = ui_recv.lineEditIP_11;
	frameAddressLineEdit[12] = ui_recv.lineEditIP_12;
	frameAddressLineEdit[13] = ui_recv.lineEditIP_13;

	frameHertzLineEdit[0] = ui_recv.lineEditHz_00;
	frameHertzLineEdit[1] = ui_recv.lineEditHz_01;
	frameHertzLineEdit[2] = ui_recv.lineEditHz_02;
	frameHertzLineEdit[3] = ui_recv.lineEditHz_03;
	frameHertzLineEdit[4] = ui_recv.lineEditHz_04;
	frameHertzLineEdit[5] = ui_recv.lineEditHz_05;
	frameHertzLineEdit[6] = ui_recv.lineEditHz_06;
	frameHertzLineEdit[7] = ui_recv.lineEditHz_07;
	frameHertzLineEdit[8] = ui_recv.lineEditHz_08;
	frameHertzLineEdit[9] = ui_recv.lineEditHz_09;
	frameHertzLineEdit[10] = ui_recv.lineEditHz_10;
	frameHertzLineEdit[11] = ui_recv.lineEditHz_11;
	frameHertzLineEdit[12] = ui_recv.lineEditHz_12;
	frameHertzLineEdit[13] = ui_recv.lineEditHz_13;

	readSettings();

	ui_recv.listViewInfo->setModel(qnode_recv->getListViewModel());

	QObject::connect(qnode_recv, SIGNAL(listViewModelUpdated()), this,
			SLOT(updateListView()));

	QObject::connect(qnode_recv, SIGNAL(frameDataUpdated()), this,
			SLOT(updateFrameViews()));
}

/**
 *
 */
WindowReceiver::~WindowReceiver() {
}

/**
 *
 * @param event
 */
void WindowReceiver::closeEvent(QCloseEvent *event) {
	writeSettings();
	QMainWindow::closeEvent(event);
}

/***********************************************
 GUI ACTION METHODS
 ***********************************************/

/**
 *
 */
void WindowReceiver::on_pushButtonReceiverSetup_clicked() {

	if (qnode_recv->readyForAction()) {

		//disable following GUI elements
		ui_recv.pushButtonReceiverSetup->setEnabled(false);

		//enable following GUI elements
		ui_recv.pushButtonReceiverStart->setEnabled(true);
		ui_recv.checkBoxPerformance->setEnabled(true);
		ui_recv.pushButtonSwitch->setEnabled(true);
		ui_recv.pushButtonResetModel->setEnabled(true);
		ui_recv.pushButtonResetFrames->setEnabled(true);

		//enable following GUI elements only when performance is unchecked
		if (!ui_recv.checkBoxPerformance->isChecked()) {
			ui_recv.checkBoxEuler->setEnabled(true);
			ui_recv.comboBoxEuler->setEnabled(true);
			ui_recv.checkBoxInactivity->setEnabled(true);
			ui_recv.checkBoxAsync->setEnabled(true);
			ui_recv.spinBoxAsync->setEnabled(true);
		}
	}
}

/**
 *
 */
void WindowReceiver::on_pushButtonReceiverStart_clicked() {

	//set node data
	qnode_recv->setSignalPerformance(ui_recv.checkBoxPerformance->isChecked());
	qnode_recv->setSignalEuler(ui_recv.checkBoxEuler->isChecked());
	qnode_recv->setFrameEuler(ui_recv.comboBoxEuler->currentIndex());
	qnode_recv->setSignalInactivity(ui_recv.checkBoxInactivity->isChecked());
	qnode_recv->setSignalAsync(ui_recv.checkBoxAsync->isChecked());
	qnode_recv->setValueAsync(ui_recv.spinBoxAsync->value());

	//start node thread
	qnode_recv->startAction();

	//disable following GUI elements
	ui_recv.pushButtonReceiverStart->setEnabled(false);
	ui_recv.checkBoxPerformance->setEnabled(false);
	ui_recv.checkBoxEuler->setEnabled(false);
	ui_recv.comboBoxEuler->setEnabled(false);
	ui_recv.checkBoxInactivity->setEnabled(false);
	ui_recv.checkBoxAsync->setEnabled(false);
	ui_recv.spinBoxAsync->setEnabled(false);

	//enable following GUI elements
	ui_recv.pushButtonReceiverStop->setEnabled(true);
}

/**
 *
 */
void WindowReceiver::on_pushButtonReceiverStop_clicked() {

	//stop node thread
	qnode_recv->stopAction();

	//disable following GUI elements
	ui_recv.pushButtonReceiverStop->setEnabled(false);
	ui_recv.pushButtonSwitch->setEnabled(false);
	ui_recv.pushButtonResetModel->setEnabled(false);
	ui_recv.pushButtonResetFrames->setEnabled(false);

	//enable following GUI elements
	ui_recv.pushButtonReceiverSetup->setEnabled(true);
	ui_recv.checkBoxPerformance->setEnabled(true);
	if(!ui_recv.checkBoxPerformance->isChecked()) {
		ui_recv.checkBoxEuler->setEnabled(true);
		ui_recv.comboBoxEuler->setEnabled(true);
		ui_recv.checkBoxInactivity->setEnabled(true);
		ui_recv.checkBoxAsync->setEnabled(true);
		ui_recv.spinBoxAsync->setEnabled(true);
	}
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBoxPerformance_stateChanged(int state) {

	//performance modus is checked
	if (state == 2) {

		//disable following GUI elements
		ui_recv.checkBoxEuler->setEnabled(false);
		ui_recv.comboBoxEuler->setEnabled(false);
		ui_recv.checkBoxInactivity->setEnabled(false);
		ui_recv.checkBoxAsync->setEnabled(false);
		ui_recv.spinBoxAsync->setEnabled(false);

		//uncheck euler checkBox
		ui_recv.checkBoxEuler->setChecked(false);
		ui_recv.checkBoxInactivity->setChecked(false);
		ui_recv.checkBoxAsync->setChecked(false);

	//performance modus is unchecked
	} else if (state == 0) {

		//enable following GUI elements
		ui_recv.checkBoxEuler->setEnabled(true);
		ui_recv.checkBoxInactivity->setEnabled(true);
		ui_recv.checkBoxAsync->setEnabled(true);

		if(ui_recv.checkBoxEuler->isChecked()) {
			ui_recv.comboBoxEuler->setEnabled(true);
		}
		if(ui_recv.checkBoxAsync->isChecked()) {
			ui_recv.spinBoxAsync->setEnabled(true);
		}
	}
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBoxEuler_stateChanged(int state) {

	//if euler is checked
	if (state == 2) {

		//enable following GUI elements
		ui_recv.comboBoxEuler->setEnabled(true);

	//if euler is unchecked
	} else if (state == 0) {

		//disable following GUI elements
		ui_recv.comboBoxEuler->setEnabled(false);
	}
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBoxAsync_stateChanged(int state) {

	//if async is checked
	if (state == 2) {

		//enable following GUI elements
		ui_recv.spinBoxAsync->setEnabled(true);

	//if async is unchecked
	} else if (state == 0) {

		//disable following GUI elements
		ui_recv.spinBoxAsync->setEnabled(false);
	}
}

/**
 *
 */
void WindowReceiver::on_pushButtonAbout_clicked() {

	dialogAbout.setModal(true);
	dialogAbout.exec();
}

/**
 *
 */
void WindowReceiver::on_pushButtonHelp_clicked() {

	dialogHelp.setModal(true);
	dialogHelp.exec();
}

/**
 *
 */
void WindowReceiver::on_pushButtonFrames_clicked() {

	dialogFrames.setModal(true);
	dialogFrames.exec();
}

/**
 *
 */
void WindowReceiver::on_pushButtonSwitch_clicked() {

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		if (i != frameSwitchComboBox[i]->currentIndex()) {
			frameSwitch(i, frameSwitchComboBox[i]->currentIndex());
			frameSwitchComboBox[i]->setCurrentIndex(i);
		}
	}
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBox_00_stateChanged(int state) {
	frameSelectChanged(0, state);
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBox_01_stateChanged(int state) {
	frameSelectChanged(1, state);
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBox_02_stateChanged(int state) {
	frameSelectChanged(2, state);
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBox_03_stateChanged(int state) {
	frameSelectChanged(3, state);
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBox_04_stateChanged(int state) {
	frameSelectChanged(4, state);
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBox_05_stateChanged(int state) {
	frameSelectChanged(5, state);
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBox_06_stateChanged(int state) {
	frameSelectChanged(6, state);
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBox_07_stateChanged(int state) {
	frameSelectChanged(7, state);
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBox_08_stateChanged(int state) {
	frameSelectChanged(8, state);
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBox_09_stateChanged(int state) {
	frameSelectChanged(9, state);
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBox_10_stateChanged(int state) {
	frameSelectChanged(10, state);
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBox_11_stateChanged(int state) {
	frameSelectChanged(11, state);
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBox_12_stateChanged(int state) {
	frameSelectChanged(12, state);
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBox_13_stateChanged(int state) {
	frameSelectChanged(13, state);
}

/**
 *
 */
void WindowReceiver::on_pushButtonResetModel_clicked() {
	QMessageBox box;
	box.setText("Reset the model?");
	box.setInformativeText(
			"Please be aware that you should not do this while recording!");
	box.setStandardButtons(QMessageBox::Apply | QMessageBox::Cancel);
	int ret = box.exec();
	switch (ret) {
	case QMessageBox::Apply:
		// Apply was clicked
		qnode_recv->setSignalResetModel(true);
		break;
	case QMessageBox::Cancel:
		// Cancel was clicked
		break;
	default:
		// should never be reached
		break;
	}
}

/**
 *
 */
void WindowReceiver::on_pushButtonResetFrames_clicked() {
	QMessageBox box;
	box.setText("Reset the frames?");
	box.setInformativeText(
			"Please be aware that you should not do this while recording!");
	box.setStandardButtons(QMessageBox::Apply | QMessageBox::Cancel);
	int ret = box.exec();
	switch (ret) {
	case QMessageBox::Apply:
		// Apply was clicked
		for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
			if (frameSelectCheckBox[i]->isChecked()) {
				qnode_recv->setFrameAddress(i, qnode_recv->getAssignAddress());
			}
			frameSelectCheckBox[i]->setChecked(true);
			frameSwitchComboBox[i]->setCurrentIndex(i);
		}
		updateFrameViews();
		break;
	case QMessageBox::Cancel:
		// Cancel was clicked
		break;
	default:
		// should never be reached
		break;
	}
}

/***********************************************
 MODEL SIGNAL METHODS
 ***********************************************/

/**
 *
 */
void WindowReceiver::updateListView() {
	ui_recv.listViewInfo->scrollToBottom();

}

/**
 *
 */
void WindowReceiver::updateFrameViews() {

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		updateFrameAddressView(i);
		updateFrameHertzView(i);
	}
}

/***********************************************
 PRIVATE GUI METHODS
 ***********************************************/

/**
 *
 * @param frame_index_a
 * @param frame_index_b
 */
void WindowReceiver::frameSwitch(const int &frame_index_a,
		const int &frame_index_b) {

	QString address_a = qnode_recv->getFrameAddress(frame_index_a);
	QString address_b = qnode_recv->getFrameAddress(frame_index_b);
	bool boolean_a = frameSelectCheckBox[frame_index_a]->isChecked();
	bool boolean_b = frameSelectCheckBox[frame_index_b]->isChecked();

	qnode_recv->setFrameAddress(frame_index_a, address_b);
	qnode_recv->setFrameAddress(frame_index_b, address_a);
	frameSelectCheckBox[frame_index_a]->setChecked(boolean_b);
	frameSelectCheckBox[frame_index_b]->setChecked(boolean_a);
	updateFrameViews();
}

/**
 *
 * @param frame_index
 * @param state
 */
void WindowReceiver::frameSelectChanged(const int &frame_index, int state) {

	if (state == 2
			&& qnode_recv->getFrameAddress(frame_index)
					== qnode_recv->getIgnoreAddress()) {
		qnode_recv->setFrameAddress(frame_index,
				qnode_recv->getAssignAddress());
	} else if (state == 0) {
		qnode_recv->setFrameAddress(frame_index,
				qnode_recv->getIgnoreAddress());
	}
	updateFrameAddressView(frame_index);
}

/**
 *
 * @param frame_index
 */
void WindowReceiver::updateFrameAddressView(const int &frame_index) {
	frameAddressLineEdit[frame_index]->setText(
			qnode_recv->getFrameAddress(frame_index));
}

/**
 *
 * @param frame_index
 */
void WindowReceiver::updateFrameHertzView(const int &frame_index) {
	frameHertzLineEdit[frame_index]->setText(
			QString::number(qnode_recv->getFrameHertzToDisplay(frame_index)));
}

/***********************************************
 GUI SETTINGS
 ***********************************************/

/**
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
		comboBoxFrames.append(qnode_recv->getFrameString(i));
	}

	//restore settings values
	ui_recv.checkBoxPerformance->setChecked(
			settings.value(QString("key_signal_performance"), true).toBool());
	ui_recv.checkBoxEuler->setChecked(
			settings.value(QString("key_signal_euler"), false).toBool());
	ui_recv.comboBoxEuler->addItems(comboBoxFrames);
	ui_recv.comboBoxEuler->setCurrentIndex(
			settings.value(QString("key_frame_euler"), 0).toInt());
	ui_recv.checkBoxInactivity->setChecked(
					settings.value(QString("key_signal_inactivity"), false).toBool());
	ui_recv.checkBoxAsync->setChecked(
				settings.value(QString("key_signal_async"), false).toBool());
	ui_recv.spinBoxAsync->setValue(
			settings.value(QString("key_value_async"), 30).toInt());

	//start reading address list
	int addressListSize = settings.beginReadArray(QString("key_address_list"));

	//restore address list if list size is correct
	if (addressListSize == NUMBER_OF_FRAMES) {
		for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
			settings.setArrayIndex(i);
			qnode_recv->addFrameAddress(
					settings.value(QString("key_address"),
							qnode_recv->getAssignAddress()).toString());
		}
	}

	//stop reading address list
	settings.endArray();

	//initialize address list if list size isn't correct
	if (addressListSize != NUMBER_OF_FRAMES) {
		for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
			qnode_recv->addFrameAddress(qnode_recv->getAssignAddress());
		}
	}

	//set values of GUI elements
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		if (qnode_recv->getFrameAddress(i) == qnode_recv->getIgnoreAddress()) {
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
	ui_recv.checkBoxPerformance->setEnabled(false);
	ui_recv.checkBoxEuler->setEnabled(false);
	ui_recv.comboBoxEuler->setEnabled(false);
	ui_recv.checkBoxInactivity->setEnabled(false);
	ui_recv.checkBoxAsync->setEnabled(false);
	ui_recv.spinBoxAsync->setEnabled(false);
	ui_recv.pushButtonReceiverStart->setEnabled(false);
	ui_recv.pushButtonReceiverStop->setEnabled(false);
	ui_recv.pushButtonSwitch->setEnabled(false);
	ui_recv.pushButtonResetModel->setEnabled(false);
	ui_recv.pushButtonResetFrames->setEnabled(false);
}

/**
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
			ui_recv.checkBoxPerformance->isChecked());
	settings.setValue(QString("key_signal_euler"),
			ui_recv.checkBoxEuler->isChecked());
	settings.setValue(QString("key_frame_euler"),
			ui_recv.comboBoxEuler->currentIndex());
	settings.setValue(QString("key_signal_inactivity"),
					ui_recv.checkBoxInactivity->isChecked());
	settings.setValue(QString("key_signal_async"),
				ui_recv.checkBoxAsync->isChecked());
	settings.setValue(QString("key_value_async"),
			ui_recv.spinBoxAsync->value());

	//start writing address list
	settings.beginWriteArray(QString("key_address_list"));

	//write address list into settings
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		settings.setArrayIndex(i);
		settings.setValue(QString("key_address"),
				qnode_recv->getFrameAddress(i));
	}

	//stop writing address list
	settings.endArray();
}
