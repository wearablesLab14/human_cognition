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
	frame_select_checkbox[0] = ui_recv.checkBox_00;
	frame_select_checkbox[1] = ui_recv.checkBox_01;
	frame_select_checkbox[2] = ui_recv.checkBox_02;
	frame_select_checkbox[3] = ui_recv.checkBox_03;
	frame_select_checkbox[4] = ui_recv.checkBox_04;
	frame_select_checkbox[5] = ui_recv.checkBox_05;
	frame_select_checkbox[6] = ui_recv.checkBox_06;
	frame_select_checkbox[7] = ui_recv.checkBox_07;
	frame_select_checkbox[8] = ui_recv.checkBox_08;
	frame_select_checkbox[9] = ui_recv.checkBox_09;
	frame_select_checkbox[10] = ui_recv.checkBox_10;
	frame_select_checkbox[11] = ui_recv.checkBox_11;
	frame_select_checkbox[12] = ui_recv.checkBox_12;
	frame_select_checkbox[13] = ui_recv.checkBox_13;

	frame_switch_combobox[0] = ui_recv.comboBox_00;
	frame_switch_combobox[1] = ui_recv.comboBox_01;
	frame_switch_combobox[2] = ui_recv.comboBox_02;
	frame_switch_combobox[3] = ui_recv.comboBox_03;
	frame_switch_combobox[4] = ui_recv.comboBox_04;
	frame_switch_combobox[5] = ui_recv.comboBox_05;
	frame_switch_combobox[6] = ui_recv.comboBox_06;
	frame_switch_combobox[7] = ui_recv.comboBox_07;
	frame_switch_combobox[8] = ui_recv.comboBox_08;
	frame_switch_combobox[9] = ui_recv.comboBox_09;
	frame_switch_combobox[10] = ui_recv.comboBox_10;
	frame_switch_combobox[11] = ui_recv.comboBox_11;
	frame_switch_combobox[12] = ui_recv.comboBox_12;
	frame_switch_combobox[13] = ui_recv.comboBox_13;

	frame_address_line[0] = ui_recv.lineEditIP_00;
	frame_address_line[1] = ui_recv.lineEditIP_01;
	frame_address_line[2] = ui_recv.lineEditIP_02;
	frame_address_line[3] = ui_recv.lineEditIP_03;
	frame_address_line[4] = ui_recv.lineEditIP_04;
	frame_address_line[5] = ui_recv.lineEditIP_05;
	frame_address_line[6] = ui_recv.lineEditIP_06;
	frame_address_line[7] = ui_recv.lineEditIP_07;
	frame_address_line[8] = ui_recv.lineEditIP_08;
	frame_address_line[9] = ui_recv.lineEditIP_09;
	frame_address_line[10] = ui_recv.lineEditIP_10;
	frame_address_line[11] = ui_recv.lineEditIP_11;
	frame_address_line[12] = ui_recv.lineEditIP_12;
	frame_address_line[13] = ui_recv.lineEditIP_13;

	frame_hertz_line[0] = ui_recv.lineEditHz_00;
	frame_hertz_line[1] = ui_recv.lineEditHz_01;
	frame_hertz_line[2] = ui_recv.lineEditHz_02;
	frame_hertz_line[3] = ui_recv.lineEditHz_03;
	frame_hertz_line[4] = ui_recv.lineEditHz_04;
	frame_hertz_line[5] = ui_recv.lineEditHz_05;
	frame_hertz_line[6] = ui_recv.lineEditHz_06;
	frame_hertz_line[7] = ui_recv.lineEditHz_07;
	frame_hertz_line[8] = ui_recv.lineEditHz_08;
	frame_hertz_line[9] = ui_recv.lineEditHz_09;
	frame_hertz_line[10] = ui_recv.lineEditHz_10;
	frame_hertz_line[11] = ui_recv.lineEditHz_11;
	frame_hertz_line[12] = ui_recv.lineEditHz_12;
	frame_hertz_line[13] = ui_recv.lineEditHz_13;

	frame_inactivity_line[0] = ui_recv.lineEditInactivity_00;
	frame_inactivity_line[1] = ui_recv.lineEditInactivity_01;
	frame_inactivity_line[2] = ui_recv.lineEditInactivity_02;
	frame_inactivity_line[3] = ui_recv.lineEditInactivity_03;
	frame_inactivity_line[4] = ui_recv.lineEditInactivity_04;
	frame_inactivity_line[5] = ui_recv.lineEditInactivity_05;
	frame_inactivity_line[6] = ui_recv.lineEditInactivity_06;
	frame_inactivity_line[7] = ui_recv.lineEditInactivity_07;
	frame_inactivity_line[8] = ui_recv.lineEditInactivity_08;
	frame_inactivity_line[9] = ui_recv.lineEditInactivity_09;
	frame_inactivity_line[10] = ui_recv.lineEditInactivity_10;
	frame_inactivity_line[11] = ui_recv.lineEditInactivity_11;
	frame_inactivity_line[12] = ui_recv.lineEditInactivity_12;
	frame_inactivity_line[13] = ui_recv.lineEditInactivity_13;

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
		ui_recv.pushButtonReceiverSetup->setEnabled(false);

		ui_recv.checkBoxEuler->setEnabled(true);
		ui_recv.comboBoxEuler->setEnabled(true);
		ui_recv.pushButtonReceiverStart->setEnabled(true);
		ui_recv.pushButtonResetModel->setEnabled(true);
		ui_recv.pushButtonResetFrames->setEnabled(true);
	}
}

/**
 *
 */
void WindowReceiver::on_pushButtonReceiverStart_clicked() {

	qnode_recv->setDisplayEulerSignal(ui_recv.checkBoxEuler->isChecked());
	qnode_recv->setDisplayEulerFrame(ui_recv.comboBoxEuler->currentIndex());

	qnode_recv->startAction();
	ui_recv.pushButtonReceiverStart->setEnabled(false);

	ui_recv.checkBoxEuler->setEnabled(false);
	ui_recv.comboBoxEuler->setEnabled(false);

	ui_recv.pushButtonReceiverStop->setEnabled(true);
	ui_recv.pushButtonResetModel->setEnabled(true);

}

/**
 *
 */
void WindowReceiver::on_pushButtonReceiverStop_clicked() {

	qnode_recv->stopAction();
	ui_recv.pushButtonReceiverStop->setEnabled(false);
	ui_recv.pushButtonResetModel->setEnabled(false);
	ui_recv.pushButtonResetFrames->setEnabled(false);

	ui_recv.pushButtonReceiverSetup->setEnabled(true);
	ui_recv.checkBoxEuler->setEnabled(true);
	ui_recv.checkBoxEuler->setChecked(false);
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBoxEuler_stateChanged(int state) {
	if (state == 2) {
		ui_recv.comboBoxEuler->setEnabled(true);
	} else if (state == 0) {
		ui_recv.comboBoxEuler->setEnabled(false);
	}
}

/**
 *
 */
void WindowReceiver::on_pushButtonRecord_clicked() {

}

/**
 *
 */
void WindowReceiver::on_pushButtonPlay_clicked() {

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
void WindowReceiver::on_pushButtonFrames_clicked() {

	dialogFrames.setModal(true);
	dialogFrames.exec();
}

/**
 *
 */
void WindowReceiver::on_pushButtonSwitch_clicked() {

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		if (i != frame_switch_combobox[i]->currentIndex()) {
			frameSwitch(i, frame_switch_combobox[i]->currentIndex());
			frame_switch_combobox[i]->setCurrentIndex(i);
		}
	}
}

/**
 *
 */
void WindowReceiver::on_pushButtonResetModel_clicked() {
	qnode_recv->setResetModelSignal(true);
}

/**
 *
 */
void WindowReceiver::on_pushButtonResetFrames_clicked() {

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		if (frame_select_checkbox[i]->isChecked()) {
			qnode_recv->setFrameAddress(i, qnode_recv->getAssignAddress());
		}
		qnode_recv->setFrameHertz(i, 0);
		qnode_recv->setLastUpdate(i, ros::Time::now());

		frame_select_checkbox[i]->setChecked(true);
		frame_switch_combobox[i]->setCurrentIndex(i);
	}
	updateFrameViews();
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
		qnode_recv->setFrameHertz(i, 0);
		updateFrameInactivityView(i);
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
	bool boolean_a = frame_select_checkbox[frame_index_a]->isChecked();
	bool boolean_b = frame_select_checkbox[frame_index_b]->isChecked();

	qnode_recv->setFrameAddress(frame_index_a, address_b);
	qnode_recv->setFrameAddress(frame_index_b, address_a);
	frame_select_checkbox[frame_index_a]->setChecked(boolean_b);
	frame_select_checkbox[frame_index_b]->setChecked(boolean_a);
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
	frame_address_line[frame_index]->setText(
			qnode_recv->getFrameAddress(frame_index));
}

/**
 *
 * @param frame_index
 */
void WindowReceiver::updateFrameHertzView(const int &frame_index) {
	frame_hertz_line[frame_index]->setText(
			QString::number(qnode_recv->getFrameHertz(frame_index)));
}

/**
 *
 * @param frame_index
 */
void WindowReceiver::updateFrameInactivityView(const int &frame_index) {
	frame_inactivity_line[frame_index]->setText(
			qnode_recv->getFrameInactivity(frame_index));
}

/***********************************************
 GUI SETTINGS
 ***********************************************/

/**
 *
 */
void WindowReceiver::readSettings() {

	QSettings settings(QString("TU Darmstadt"),
			QString("human_cognition_receiver"));

	restoreGeometry(settings.value(QString("geometry")).toByteArray());
	restoreState(settings.value(QString("windowState")).toByteArray());

	qnode_recv->clearAllFrameAddresses();
	int size = settings.beginReadArray("addressList");
	if (size == NUMBER_OF_FRAMES) {
		for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
			settings.setArrayIndex(i);
			qnode_recv->addFrameAddress(
					settings.value("address", qnode_recv->getAssignAddress()).toString());
		}
		settings.endArray();
	} else {
		settings.endArray();
		for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
			qnode_recv->addFrameAddress(qnode_recv->getAssignAddress());
		}
	}

	QStringList comboList;
	comboList.clear();
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		if (i < 10) {
			comboList.append(QString("0%1").arg(i));
		} else {
			comboList.append(QString("%1").arg(i));
		}
	}
	ui_recv.comboBoxEuler->addItems(comboList);
	ui_recv.comboBoxEuler->setEnabled(false);

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		frame_switch_combobox[i]->addItems(comboList);
		frame_switch_combobox[i]->setCurrentIndex(i);
	}

	ui_recv.checkBoxEuler->setChecked(false);

	ui_recv.pushButtonReceiverStart->setEnabled(false);
	ui_recv.pushButtonReceiverStop->setEnabled(false);
	ui_recv.pushButtonResetModel->setEnabled(false);
	ui_recv.pushButtonResetFrames->setEnabled(false);

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

		if (qnode_recv->getFrameAddress(i) == qnode_recv->getIgnoreAddress()) {
			frame_select_checkbox[i]->setChecked(false);
		} else {
			frame_select_checkbox[i]->setChecked(true);
		}
		updateFrameAddressView(i);
		updateFrameHertzView(i);
		updateFrameInactivityView(i);
	}
}

/**
 *
 */
void WindowReceiver::writeSettings() {
	QSettings settings(QString("TU Darmstadt"),
			QString("human_cognition_receiver"));
	settings.setValue(QString("geometry"), saveGeometry());
	settings.setValue(QString("windowState"), saveState());

	settings.beginWriteArray("addressList");
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		settings.setArrayIndex(i);
		settings.setValue("address", qnode_recv->getFrameAddress(i));
	}
	settings.endArray();
}
