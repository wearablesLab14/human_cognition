#ifndef human_cognition_WINDOW_RECEIVER_H
#define human_cognition_WINDOW_RECEIVER_H

#include <QtGui>
#include <QtGui/QDialog>
#include <QtGui/QMainWindow>

#ifndef Q_MOC_RUN
#include "ui_window_receiver.h"
#include "dialog_about.hpp"
#include "dialog_frames.hpp"
#include "../node_receiver/qnode_receiver.hpp"
#endif

/**
 *
 */
class WindowReceiver: public QMainWindow {
Q_OBJECT

public:
	WindowReceiver(QNodeReceiver *node, QWidget *parent = 0);
	~WindowReceiver();
	void closeEvent(QCloseEvent *event);

public Q_SLOTS:
	/***********************************************
	 MODEL SIGNAL METHODS
	 ***********************************************/
	void updateListView();
	void updateFrameViews();

	/***********************************************
	 GUI ACTION METHODS
	 ***********************************************/
	void on_pushButtonReceiverSetup_clicked();
	void on_pushButtonReceiverStart_clicked();
	void on_pushButtonReceiverStop_clicked();
	void on_checkBoxEuler_stateChanged(int state);
	void on_pushButtonRecord_clicked();
	void on_pushButtonPlay_clicked();
	void on_pushButtonAbout_clicked();
	void on_pushButtonFrames_clicked();
	void on_pushButtonSwitch_clicked();
	void on_checkBox_00_stateChanged(int state);
	void on_checkBox_01_stateChanged(int state);
	void on_checkBox_02_stateChanged(int state);
	void on_checkBox_03_stateChanged(int state);
	void on_checkBox_04_stateChanged(int state);
	void on_checkBox_05_stateChanged(int state);
	void on_checkBox_06_stateChanged(int state);
	void on_checkBox_07_stateChanged(int state);
	void on_checkBox_08_stateChanged(int state);
	void on_checkBox_09_stateChanged(int state);
	void on_checkBox_10_stateChanged(int state);
	void on_checkBox_11_stateChanged(int state);
	void on_checkBox_12_stateChanged(int state);
	void on_checkBox_13_stateChanged(int state);
	void on_pushButtonResetModel_clicked();
	void on_pushButtonResetFrames_clicked();

private:
	/***********************************************
	 PRIVATE GUI METHODS
	 ***********************************************/
	void frameSwitch(const int &frame_index_a, const int &frame_index_b);
	void frameSelectChanged(const int &frame_index, int state);
	void updateFrameAddressView(const int &frame_index);
	void updateFrameHertzView(const int &frame_index);
	void updateFrameInactivityView(const int &frame_index);

	/***********************************************
	 GUI SETTINGS
	 ***********************************************/
	void readSettings();
	void writeSettings();

	Ui::WindowReceiverDesign ui_recv;
	QNodeReceiver *qnode_recv;
	DialogAbout dialogAbout;
	DialogFrames dialogFrames;

	QCheckBox* frame_select_checkbox[NUMBER_OF_FRAMES];
	QComboBox* frame_switch_combobox[NUMBER_OF_FRAMES];
	QLineEdit* frame_address_line[NUMBER_OF_FRAMES];
	QLineEdit* frame_hertz_line[NUMBER_OF_FRAMES];
	QLineEdit* frame_inactivity_line[NUMBER_OF_FRAMES];
};

#endif
