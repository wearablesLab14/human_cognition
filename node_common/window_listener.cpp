/****************************************************************
 *  Project:
 *  	Integrating Body- and Eye-Tracking to study Cognition in the Wild
 *	-------------------------------------------------------------
 * 	TU Darmstadt
 * 	Department Computer Science
 * 	Summer term 2014
 *	-------------------------------------------------------------
 *	File: window_listener.cpp
 *	Description:
 *		GUI class for listener node
 *
 *	-------------------------------------------------------------
 * 	Authors:
 * 		Christian Benz 			<zneb_naitsirhc@web.de>
 * 		Christoph Döringer 		<christoph.doeringer@gmail.com>
 * 		Hendrik Pfeifer 		<hendrikpfeifer@gmail.com>
 * 		Heiko Reinemuth 		<heiko.reinemuth@gmail.com>
 ****************************************************************/

#include "window_listener.hpp"

using namespace Qt;

/*! \brief Constructor of WindowListener class
 *
 * @param node The node class for this GUI class
 * @param parent Parent for this window
 */
WindowListener::WindowListener(QNodeListener *node, QWidget *parent) :
		QMainWindow(parent), qnodeListener(node) {

	//setup user interface
	uiListener.setupUi(this);

	//restore GUI state and values from QSettings
	readSettings();

	//set listView model
	uiListener.listViewInfo->setModel(qnodeListener->getListViewModel());

	//connect signal call of node with GUI update method
	QObject::connect(qnodeListener, SIGNAL(listViewModelUpdated()), this,
			SLOT(updateListView()));
}

/*! \brief Destructor of WindowListener class
 *
 */
WindowListener::~WindowListener() {
}

/*! \brief Saves GUI state and values on window close event
 *
 * @param event Window closing event
 */
void WindowListener::closeEvent(QCloseEvent *event) {

	//save GUI state and values in QSettings
	writeSettings();

	//close window
	QMainWindow::closeEvent(event);
}

/***********************************************
 GUI ACTION METHODS
 ***********************************************/

/*! \brief Initializes node for listening, enables and disables GUI elements
 *
 */
void WindowListener::on_pushButtonListenerSetup_clicked() {

	//initialize  listener node for listening
	if (qnodeListener->readyForAction()) {

		//disable following GUI elements
		uiListener.pushButtonListenerSetup->setEnabled(false);

		//enable following GUI elements
		uiListener.pushButtonListenerStart->setEnabled(true);
		uiListener.checkBoxCoordinates->setEnabled(true);
		uiListener.comboBoxCoordinates->setEnabled(true);
		uiListener.checkBoxRecord->setEnabled(true);
		uiListener.lineEditRecord->setEnabled(true);
	}
}

/*! \brief Starts thread for receiving, enables and disables GUI elements
 *
 */
void WindowListener::on_pushButtonListenerStart_clicked() {

	//set node data
	qnodeListener->setSignalCoordinates(
			uiListener.checkBoxCoordinates->isChecked());
	qnodeListener->setFrameCoordinates(
			uiListener.comboBoxCoordinates->currentIndex());
	qnodeListener->setSignalRecord(uiListener.checkBoxRecord->isChecked());
	qnodeListener->setFileRecord(
			uiListener.lineEditRecord->text().toStdString());

	//start node thread
	qnodeListener->startAction();

	//disable following GUI elements
	uiListener.pushButtonListenerStart->setEnabled(false);
	uiListener.checkBoxCoordinates->setEnabled(false);
	uiListener.comboBoxCoordinates->setEnabled(false);
	uiListener.checkBoxRecord->setEnabled(false);
	uiListener.lineEditRecord->setEnabled(false);

	//enable following GUI elements
	uiListener.pushButtonListenerStop->setEnabled(true);
}

/*! \brief Stops thread for listening, enables and disables GUI elements
 *
 */
void WindowListener::on_pushButtonListenerStop_clicked() {

	//stop node thread
	qnodeListener->stopAction();

	//disable following GUI elements
	uiListener.pushButtonListenerStop->setEnabled(false);

	//enable following GUI elements
	uiListener.pushButtonListenerSetup->setEnabled(true);
}

/*! \brief Enables and disables the coordinates comboBox depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowListener::on_checkBoxDisplayCoordinates_stateChanged(int state) {

	//if display coordinates is checked
	if (state == 2) {

		//enable following GUI element
		uiListener.comboBoxCoordinates->setEnabled(true);

		//if display coordinates is unchecked
	} else if (state == 0) {

		//disable following GUI element
		uiListener.comboBoxCoordinates->setEnabled(false);
	}
}

/*! \brief Enables and disables the record file lineEdit depending on the changed state
 *
 * @param state The new state of the checkBox
 */
void WindowListener::on_checkBoxRecordCoordinates_stateChanged(int state) {

	//if record coordinates is checked
	if (state == 2) {

		//enable following GUI element
		uiListener.lineEditRecord->setEnabled(true);

		//if record coordinates is unchecked
	} else if (state == 0) {

		//disable following GUI element
		uiListener.lineEditRecord->setEnabled(false);
	}
}

/***********************************************
 MODEL SIGNAL METHODS
 ***********************************************/

/*! \brief Scrolls listView to the bottom
 *
 */
void WindowListener::updateListView() {
	uiListener.listViewInfo->scrollToBottom();
}

/***********************************************
 PRIVATE GUI METHODS
 ***********************************************/

/*! \brief Restores GUI state and values from QSettings
 *
 */
void WindowListener::readSettings() {

	//listener settings
	QSettings settings(QString("TU Darmstadt"),
			QString("human_cognition_listener"));

	//restore geometry and window state
	restoreGeometry(settings.value(QString("key_geometry")).toByteArray());
	restoreState(settings.value(QString("key_window_state")).toByteArray());

	//initialize standard frames list for comboBoxes
	QStringList comboBoxFrames;
	for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
		comboBoxFrames.append(qnodeListener->getFrameString(i));
	}

	//restore settings values
	uiListener.checkBoxCoordinates->setChecked(
			settings.value(QString("key_signal_coordinates"), true).toBool());
	uiListener.comboBoxCoordinates->addItems(comboBoxFrames);
	uiListener.comboBoxCoordinates->setCurrentIndex(
			settings.value(QString("key_frame_coordinates"), 0).toInt());
	uiListener.checkBoxRecord->setChecked(
			settings.value(QString("key_signal_record"), true).toBool());
	uiListener.lineEditRecord->setText(
			settings.value(QString("key_file_record"),
					QString("coordinates.csv")).toString());

	//disable following GUI elements
	uiListener.pushButtonListenerStart->setEnabled(false);
	uiListener.pushButtonListenerStop->setEnabled(false);
	uiListener.checkBoxCoordinates->setEnabled(false);
	uiListener.comboBoxCoordinates->setEnabled(false);
	uiListener.checkBoxRecord->setEnabled(false);
	uiListener.lineEditRecord->setEnabled(false);
}

/*! \brief Saves GUI state and values in QSettings
 *
 */
void WindowListener::writeSettings() {

	//listener settings
	QSettings settings(QString("TU Darmstadt"),
			QString("human_cognition_listener"));

	//save geometry and window state
	settings.setValue(QString("key_geometry"), saveGeometry());
	settings.setValue(QString("key_window_state"), saveState());

	//save settings values
	settings.setValue(QString("key_signal_coordinates"),
			uiListener.checkBoxCoordinates->isChecked());
	settings.setValue(QString("key_frame_coordinates"),
			uiListener.comboBoxCoordinates->currentIndex());
	settings.setValue(QString("key_signal_record"),
			uiListener.checkBoxRecord->isChecked());
	settings.setValue(QString("key_file_record"),
			uiListener.lineEditRecord->text());
}
