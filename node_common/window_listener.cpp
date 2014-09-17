#include "window_listener.hpp"

using namespace Qt;

/**
 *
 * @param node
 * @param parent
 */
WindowListener::WindowListener(QNodeListener *node, QWidget *parent) :
		QMainWindow(parent), qnode_list(node) {

	ui_list.setupUi(this);

	readSettings();

	ui_list.listViewInfo->setModel(qnode_list->getListViewModel());

	QObject::connect(qnode_list, SIGNAL(listViewModelUpdated()), this, SLOT(updateListView()));
}

/**
 *
 */
WindowListener::~WindowListener() {
}

/**
 *
 * @param event
 */
void WindowListener::closeEvent(QCloseEvent *event) {
	writeSettings();
	QMainWindow::closeEvent(event);
}

/***********************************************
 GUI ACTION METHODS
 ***********************************************/

/**
 *
 */
void WindowListener::on_pushButtonListenerSetup_clicked() {

	if (qnode_list->readyForAction()) {
		ui_list.pushButtonListenerSetup->setEnabled(false);

		ui_list.pushButtonListenerStart->setEnabled(true);
		ui_list.checkBoxDisplayCoordinates->setEnabled(true);
		ui_list.comboBoxCoordinates->setEnabled(true);
		ui_list.checkBoxRecordCoordinates->setEnabled(true);
		ui_list.lineEditFile->setEnabled(true);
	}
}

/**
 *
 */
void WindowListener::on_pushButtonListenerStart_clicked() {

	qnode_list->setDisplayCoordinatesSignal(ui_list.checkBoxDisplayCoordinates->isChecked());
	qnode_list->setDisplayCoordinatesFrame(ui_list.comboBoxCoordinates->currentIndex());


	qnode_list->setRecordCoordinatesSignal(ui_list.checkBoxRecordCoordinates->isChecked());
	qnode_list->setRecordCoordinatesFile(ui_list.lineEditFile->text().toStdString());

	qnode_list->startAction();
	ui_list.pushButtonListenerStart->setEnabled(false);

	ui_list.checkBoxDisplayCoordinates->setEnabled(false);
	ui_list.comboBoxCoordinates->setEnabled(false);
	ui_list.checkBoxRecordCoordinates->setEnabled(false);
	ui_list.lineEditFile->setEnabled(false);

	ui_list.pushButtonListenerStop->setEnabled(true);
}

/**
 *
 */
void WindowListener::on_pushButtonListenerStop_clicked() {

	qnode_list->stopAction();
	ui_list.pushButtonListenerStop->setEnabled(false);

	ui_list.pushButtonListenerSetup->setEnabled(true);

	ui_list.checkBoxDisplayCoordinates->setChecked(false);
	ui_list.checkBoxRecordCoordinates->setChecked(false);
}

/**
 *
 * @param state
 */
void WindowListener::on_checkBoxDisplayCoordinates_stateChanged(int state) {
	if (state == 2) {
		ui_list.comboBoxCoordinates->setEnabled(true);
	} else if (state == 0) {
		ui_list.comboBoxCoordinates->setEnabled(false);
	}
}

/**
 *
 * @param state
 */
void WindowListener::on_checkBoxRecordCoordinates_stateChanged(int state) {
	if (state == 2) {
		ui_list.lineEditFile->setEnabled(true);
	} else if (state == 0) {
		ui_list.lineEditFile->setEnabled(false);
	}
}

/***********************************************
 MODEL SIGNAL METHODS
 ***********************************************/

/*
/**
 *
 */
void WindowListener::updateListView() {
	ui_list.listViewInfo->scrollToBottom();

}

/***********************************************
 GUI SETTINGS
 ***********************************************/

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

	ui_list.lineEditFile->setText("coordinates.csv");
	ui_list.comboBoxCoordinates->addItems(comboList);
	ui_list.comboBoxCoordinates->setEnabled(false);

	ui_list.checkBoxDisplayCoordinates->setChecked(false);
	ui_list.checkBoxRecordCoordinates->setChecked(false);

	ui_list.pushButtonListenerStart->setEnabled(false);
	ui_list.pushButtonListenerStop->setEnabled(false);
}

/**
 *
 */
void WindowListener::writeSettings() {
	QSettings settings(QString("TU Darmstadt"), QString("human_cognition_listener"));
	settings.setValue(QString("geometry"), saveGeometry());
	settings.setValue(QString("windowState"), saveState());
}
