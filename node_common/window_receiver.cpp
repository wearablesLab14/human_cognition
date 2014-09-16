#include "window_receiver.hpp"
//#include "about_window.hpp"

using namespace Qt;

/**
 *
 * @param argc
 * @param argv
 * @param parent
 */
WindowReceiver::WindowReceiver(QNode *node, QWidget *parent) :
		QMainWindow(parent), qnode(node) {

	ui.setupUi(this);
	initQElements();

	readSettings();

	ui.listViewInfo->setModel(qnode->getListViewModel());

	QObject::connect(qnode, SIGNAL(frameInfoUpdated()), this,
			SLOT(updateFrameInfo()));

	QObject::connect(qnode, SIGNAL(listInfoUpdated()), this,
			SLOT(updateListInfo()));
}

/**
 *
 */
WindowReceiver::~WindowReceiver() {
}

//**************************************************************************************

void WindowReceiver::on_pushButtonAbout_clicked() {

	/*about = new About(this);
	about->exec();*/
}

/**
 *
 */
void WindowReceiver::initQElements() {

	frame_select_checkbox[0] = ui.checkBox_00;
	frame_select_checkbox[1] = ui.checkBox_01;
	frame_select_checkbox[2] = ui.checkBox_02;
	frame_select_checkbox[3] = ui.checkBox_03;
	frame_select_checkbox[4] = ui.checkBox_04;
	frame_select_checkbox[5] = ui.checkBox_05;
	frame_select_checkbox[6] = ui.checkBox_06;
	frame_select_checkbox[7] = ui.checkBox_07;
	frame_select_checkbox[8] = ui.checkBox_08;
	frame_select_checkbox[9] = ui.checkBox_09;
	frame_select_checkbox[10] = ui.checkBox_10;
	frame_select_checkbox[11] = ui.checkBox_11;
	frame_select_checkbox[12] = ui.checkBox_12;
	frame_select_checkbox[13] = ui.checkBox_13;

	frame_switch_combobox[0] = ui.comboBox_00;
	frame_switch_combobox[1] = ui.comboBox_01;
	frame_switch_combobox[2] = ui.comboBox_02;
	frame_switch_combobox[3] = ui.comboBox_03;
	frame_switch_combobox[4] = ui.comboBox_04;
	frame_switch_combobox[5] = ui.comboBox_05;
	frame_switch_combobox[6] = ui.comboBox_06;
	frame_switch_combobox[7] = ui.comboBox_07;
	frame_switch_combobox[8] = ui.comboBox_08;
	frame_switch_combobox[9] = ui.comboBox_09;
	frame_switch_combobox[10] = ui.comboBox_10;
	frame_switch_combobox[11] = ui.comboBox_11;
	frame_switch_combobox[12] = ui.comboBox_12;
	frame_switch_combobox[13] = ui.comboBox_13;

	frame_address_line[0] = ui.lineEditIP_00;
	frame_address_line[1] = ui.lineEditIP_01;
	frame_address_line[2] = ui.lineEditIP_02;
	frame_address_line[3] = ui.lineEditIP_03;
	frame_address_line[4] = ui.lineEditIP_04;
	frame_address_line[5] = ui.lineEditIP_05;
	frame_address_line[6] = ui.lineEditIP_06;
	frame_address_line[7] = ui.lineEditIP_07;
	frame_address_line[8] = ui.lineEditIP_08;
	frame_address_line[9] = ui.lineEditIP_09;
	frame_address_line[10] = ui.lineEditIP_10;
	frame_address_line[11] = ui.lineEditIP_11;
	frame_address_line[12] = ui.lineEditIP_12;
	frame_address_line[13] = ui.lineEditIP_13;

	frame_hertz_line[0] = ui.lineEditHz_00;
	frame_hertz_line[1] = ui.lineEditHz_01;
	frame_hertz_line[2] = ui.lineEditHz_02;
	frame_hertz_line[3] = ui.lineEditHz_03;
	frame_hertz_line[4] = ui.lineEditHz_04;
	frame_hertz_line[5] = ui.lineEditHz_05;
	frame_hertz_line[6] = ui.lineEditHz_06;
	frame_hertz_line[7] = ui.lineEditHz_07;
	frame_hertz_line[8] = ui.lineEditHz_08;
	frame_hertz_line[9] = ui.lineEditHz_09;
	frame_hertz_line[10] = ui.lineEditHz_10;
	frame_hertz_line[11] = ui.lineEditHz_11;
	frame_hertz_line[12] = ui.lineEditHz_12;
	frame_hertz_line[13] = ui.lineEditHz_13;

	frame_inactivity_line[0] = ui.lineEditInactivity_00;
	frame_inactivity_line[1] = ui.lineEditInactivity_01;
	frame_inactivity_line[2] = ui.lineEditInactivity_02;
	frame_inactivity_line[3] = ui.lineEditInactivity_03;
	frame_inactivity_line[4] = ui.lineEditInactivity_04;
	frame_inactivity_line[5] = ui.lineEditInactivity_05;
	frame_inactivity_line[6] = ui.lineEditInactivity_06;
	frame_inactivity_line[7] = ui.lineEditInactivity_07;
	frame_inactivity_line[8] = ui.lineEditInactivity_08;
	frame_inactivity_line[9] = ui.lineEditInactivity_09;
	frame_inactivity_line[10] = ui.lineEditInactivity_10;
	frame_inactivity_line[11] = ui.lineEditInactivity_11;
	frame_inactivity_line[12] = ui.lineEditInactivity_12;
	frame_inactivity_line[13] = ui.lineEditInactivity_13;
}

//**************************************************************************************

/**
 *
 */
void WindowReceiver::on_pushButtonReceiverSetup_clicked() {

	if (qnode->receiveReady()) {
		ui.pushButtonReceiverSetup->setEnabled(false);

		ui.checkBoxEuler->setEnabled(true);
		ui.comboBoxEuler->setEnabled(true);
		ui.pushButtonReceiverStart->setEnabled(true);
		ui.pushButtonResetModel->setEnabled(true);
		ui.pushButtonResetFrames->setEnabled(true);
	}
}

/**
 *
 */
void WindowReceiver::on_pushButtonReceiverStart_clicked() {

	qnode->setDisplayEulerSignal(ui.checkBoxEuler->isChecked());
	qnode->setDisplayEulerFrame(ui.comboBoxEuler->currentIndex());

	qnode->startThread();
	ui.pushButtonReceiverStart->setEnabled(false);


	ui.checkBoxEuler->setEnabled(false);
	ui.comboBoxEuler->setEnabled(false);

	ui.pushButtonReceiverStop->setEnabled(true);
	ui.pushButtonResetModel->setEnabled(true);

}

/**
 *
 */
void WindowReceiver::on_pushButtonReceiverStop_clicked() {

	qnode->stopThread();
	ui.pushButtonReceiverStop->setEnabled(false);
	ui.pushButtonResetModel->setEnabled(false);
	ui.pushButtonResetFrames->setEnabled(false);

	ui.pushButtonReceiverSetup->setEnabled(true);
	ui.checkBoxEuler->setEnabled(true);
	ui.checkBoxEuler->setChecked(false);
}

/**
 *
 */
void WindowReceiver::on_pushButtonSwitch_clicked() {

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		if (i != frame_switch_combobox[i]->currentIndex()) {
			switchFrames(i, frame_switch_combobox[i]->currentIndex());
			frame_switch_combobox[i]->setCurrentIndex(i);
		}
	}
}

/**
 *
 */
void WindowReceiver::on_pushButtonResetModel_clicked() {
	qnode->setResetModelSignal(true);
}

/**
 *
 */
void WindowReceiver::on_pushButtonResetFrames_clicked() {
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

		if (frame_select_checkbox[i]->isChecked()) {
			qnode->setFrameAddress(i, qnode->getAssignAddress());
		}
		qnode->setFrameHertz(i, 0);
		qnode->setLastUpdate(i, ros::Time::now());

		frame_select_checkbox[i]->setChecked(true);
		frame_switch_combobox[i]->setCurrentIndex(i);
	}
	updateFrameInfo();
}

/**
 *
 * @param state
 */
void WindowReceiver::on_checkBoxEuler_stateChanged(int state) {
	if (state == 2) {
		ui.comboBoxEuler->setEnabled(true);
	} else if (state == 0) {
		ui.comboBoxEuler->setEnabled(false);
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

//**************************************************************************************

/**
 *
 * @param index_a
 * @param index_b
 */
void WindowReceiver::switchFrames(int index_a, int index_b) {

	QString address_a = qnode->getFrameAddress(index_a);
	QString address_b = qnode->getFrameAddress(index_b);
	bool boolean_a = frame_select_checkbox[index_a]->isChecked();
	bool boolean_b = frame_select_checkbox[index_b]->isChecked();

	qnode->setFrameAddress(index_a, address_b);
	qnode->setFrameAddress(index_b, address_a);
	frame_select_checkbox[index_a]->setChecked(boolean_b);
	frame_select_checkbox[index_b]->setChecked(boolean_a);
	updateFrameInfo();
}

//**************************************************************************************

/**
 *
 */
void WindowReceiver::updateListInfo() {
	ui.listViewInfo->scrollToBottom();

}

//**************************************************************************************

/**
 *
 */
void WindowReceiver::updateFrameInfo() {

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		updateFrameIP(i);
		updateFrameHertz(i);
		qnode->setFrameHertz(i, 0);
		updateFrameInactivity(i);
		updateFrameColor(i);
	}
}

/**
 *
 * @param index
 */
void WindowReceiver::updateFrameColor(int index) {

	QPalette *ignorePalette = new QPalette();
	QPalette *assignPalette = new QPalette();
	QPalette *activePalette = new QPalette();
	ignorePalette->setColor(QPalette::Text, Qt::darkGray);
	assignPalette->setColor(QPalette::Text, Qt::darkMagenta);
	activePalette->setColor(QPalette::Text, Qt::darkCyan);

	if (qnode->getFrameAddress(index) == qnode->getIgnoreAddress()) {
		frame_address_line[index]->setPalette(*ignorePalette);
		frame_hertz_line[index]->setPalette(*ignorePalette);
		frame_inactivity_line[index]->setPalette(*ignorePalette);
	} else if (qnode->getFrameAddress(index) == qnode->getAssignAddress()) {
		frame_address_line[index]->setPalette(*assignPalette);
		frame_hertz_line[index]->setPalette(*assignPalette);
		frame_inactivity_line[index]->setPalette(*assignPalette);
	} else {
		frame_address_line[index]->setPalette(*activePalette);
		frame_hertz_line[index]->setPalette(*activePalette);
		frame_inactivity_line[index]->setPalette(*activePalette);
	}
}

/**
 *
 * @param index
 */
void WindowReceiver::updateFrameIP(int index) {
	frame_address_line[index]->setText(qnode->getFrameAddress(index));
}

/**
 *
 * @param index
 */
void WindowReceiver::updateFrameHertz(const int &frame_index) {
	frame_hertz_line[frame_index]->setText(
			QString::number(qnode->getFrameHertz(frame_index)));
}

/**
 *
 * @param index
 */
void WindowReceiver::updateFrameInactivity(const int &frame_index) {
	frame_inactivity_line[frame_index]->setText(qnode->getFrameInactivity(frame_index));
}

/**
 *
 * @param index
 * @param state
 */
void WindowReceiver::frameSelectChanged(int index, int state) {

	if (state == 2 && qnode->getFrameAddress(index) == qnode->getIgnoreAddress()) {
		qnode->setFrameAddress(index, qnode->getAssignAddress());
	} else if (state == 0) {
		qnode->setFrameAddress(index, qnode->getIgnoreAddress());
	}
	updateFrameIP(index);
	updateFrameColor(index);
}

//**************************************************************************************

/**
 *
 */
void WindowReceiver::readSettings() {

	QSettings settings(QString("TU Darmstadt"), QString("cognition_project"));

	restoreGeometry(settings.value(QString("geometry")).toByteArray());
	restoreState(settings.value(QString("windowState")).toByteArray());

	qnode->clearAllFrameAddesses();
	int size = settings.beginReadArray("addressList");
	if (size == NUMBER_OF_FRAMES) {
		for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
			settings.setArrayIndex(i);
			qnode->addFrameAddress(
					settings.value("address", qnode->getAssignAddress()).toString());
		}
		settings.endArray();
	} else {
		settings.endArray();
		for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
			qnode->addFrameAddress(qnode->getAssignAddress());
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
	ui.comboBoxEuler->addItems(comboList);
	ui.comboBoxEuler->setEnabled(false);

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		frame_switch_combobox[i]->addItems(comboList);
		frame_switch_combobox[i]->setCurrentIndex(i);
	}

	ui.checkBoxEuler->setChecked(false);

	ui.pushButtonReceiverStart->setEnabled(false);
	ui.pushButtonReceiverStop->setEnabled(false);
	ui.pushButtonResetModel->setEnabled(false);
	ui.pushButtonResetFrames->setEnabled(false);

	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {

		if (qnode->getFrameAddress(i) == qnode->getIgnoreAddress()) {
			frame_select_checkbox[i]->setChecked(false);
		} else {
			frame_select_checkbox[i]->setChecked(true);
		}
		updateFrameIP(i);
		updateFrameHertz(i);
		updateFrameInactivity(i);
		updateFrameColor(i);
	}
}

/**
 *
 */
void WindowReceiver::writeSettings() {
	QSettings settings(QString("TU Darmstadt"), QString("cognition_project"));
	settings.setValue(QString("geometry"), saveGeometry());
	settings.setValue(QString("windowState"), saveState());

	settings.beginWriteArray("addressList");
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		settings.setArrayIndex(i);
		settings.setValue("address", qnode->getFrameAddress(i));
	}
	settings.endArray();
}

//**************************************************************************************

/**
 *
 * @param event
 */
void WindowReceiver::closeEvent(QCloseEvent *event) {
	writeSettings();
	QMainWindow::closeEvent(event);
}

