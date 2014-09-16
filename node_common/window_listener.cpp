#include "window_listener.hpp"

using namespace Qt;

/**
 *
 * @param argc
 * @param argv
 * @param parent
 */
WindowListener::WindowListener(QNode *node, QWidget *parent) :
		QMainWindow(parent), qnode(node) {

	ui.setupUi(this);

	readSettings();

	ui.listViewInfo->setModel(qnode->getListViewModel());

	QObject::connect(qnode, SIGNAL(listInfoUpdated()), this, SLOT(updateListInfo()));
}

/**
 *
 */
WindowListener::~WindowListener() {
}

//**************************************************************************************

/**
 *
 */
void WindowListener::on_pushButtonListenerSetup_clicked() {

	if (qnode->listenReady()) {
		ui.pushButtonListenerSetup->setEnabled(false);

		ui.pushButtonListenerStart->setEnabled(true);
		ui.checkBoxDisplayCoordinates->setEnabled(true);
		ui.comboBoxCoordinates->setEnabled(true);
		ui.checkBoxRecordCoordinates->setEnabled(true);
		ui.lineEditFile->setEnabled(true);
	}

}

/**
 *
 */
void WindowListener::on_pushButtonListenerStart_clicked() {

	qnode->setDisplayCoordinatesSignal(ui.checkBoxDisplayCoordinates->isChecked());
	qnode->setDisplayCoordinatesFrame(ui.comboBoxCoordinates->currentIndex());


	qnode->setRecordCoordinatesSignal(ui.checkBoxRecordCoordinates->isChecked());
	qnode->setRecordCoordinatesFile(ui.lineEditFile->text().toStdString());

	qnode->startThread();
	ui.pushButtonListenerStart->setEnabled(false);

	ui.checkBoxDisplayCoordinates->setEnabled(false);
	ui.comboBoxCoordinates->setEnabled(false);
	ui.checkBoxRecordCoordinates->setEnabled(false);
	ui.lineEditFile->setEnabled(false);

	ui.pushButtonListenerStop->setEnabled(true);
}

/**
 *
 */
void WindowListener::on_pushButtonListenerStop_clicked() {

	qnode->stopThread();
	ui.pushButtonListenerStop->setEnabled(false);

	ui.pushButtonListenerSetup->setEnabled(true);

	ui.checkBoxDisplayCoordinates->setChecked(false);
	ui.checkBoxRecordCoordinates->setChecked(false);
}

/**
 *
 * @param state
 */
void WindowListener::on_checkBoxDisplayCoordinates_stateChanged(int state) {
	if (state == 2) {
		ui.comboBoxCoordinates->setEnabled(true);
	} else if (state == 0) {
		ui.comboBoxCoordinates->setEnabled(false);
	}
}

/**
 *
 * @param state
 */
void WindowListener::on_checkBoxRecordCoordinates_stateChanged(int state) {
	if (state == 2) {
		ui.lineEditFile->setEnabled(true);
	} else if (state == 0) {
		ui.lineEditFile->setEnabled(false);
	}
}

/*
/**
 *
 */
void WindowListener::updateListInfo() {
	ui.listViewInfo->scrollToBottom();

}

/**
 *
 */
void WindowListener::readSettings() {

	QSettings settings(QString("TU Darmstadt"), QString("human_cognition_listener"));

	restoreGeometry(settings.value(QString("geometry")).toByteArray());
	restoreState(settings.value(QString("windowState")).toByteArray());

	QStringList comboList;
	comboList.clear();
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		if (i < 10) {
			comboList.append(QString("0%1").arg(i));
		} else {
			comboList.append(QString("%1").arg(i));
		}
	}

	ui.lineEditFile->setText("coordinates.csv");
	ui.comboBoxCoordinates->addItems(comboList);
	ui.comboBoxCoordinates->setEnabled(false);

	ui.checkBoxDisplayCoordinates->setChecked(false);
	ui.checkBoxRecordCoordinates->setChecked(false);

	ui.pushButtonListenerStart->setEnabled(false);
	ui.pushButtonListenerStop->setEnabled(false);
}

/**
 *
 */
void WindowListener::writeSettings() {
	QSettings settings(QString("TU Darmstadt"), QString("human_cognition_listener"));
	settings.setValue(QString("geometry"), saveGeometry());
	settings.setValue(QString("windowState"), saveState());
}

/**
 *
 * @param event
 */
void WindowListener::closeEvent(QCloseEvent *event) {
	writeSettings();
	QMainWindow::closeEvent(event);
}

