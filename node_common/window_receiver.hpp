#ifndef human_cognition_WINDOW_RECEIVER_H
#define human_cognition_WINDOW_RECEIVER_H

#include <QtGui>
#include <QtGui/QMainWindow>

#ifndef Q_MOC_RUN
#include "node_common/ui_window_receiver.h"
#include "node_common/qnode.hpp"
//#include "node_common/about_window.hpp"
#endif

class WindowReceiver: public QMainWindow {
Q_OBJECT

public:
	WindowReceiver(QNode *node, QWidget *parent = 0);
	~WindowReceiver();
	void closeEvent(QCloseEvent *event);

	void initQElements();
	void switchFrames(int index_a, int index_b);
	void frameSelectChanged(int index, int state);
	void updateFrameColor(int index);
	void updateFrameIP(int index);
	void updateFrameHertz(const int &frame_index);
	void updateFrameInactivity(const int &frame_index);
	void resetFrames();
	void readSettings();
	void writeSettings();

public Q_SLOTS:
	void updateListInfo();
	void updateFrameInfo();

	void on_pushButtonReceiverSetup_clicked();
	void on_pushButtonReceiverStart_clicked();
	void on_pushButtonReceiverStop_clicked();
	//void on_pushButtonRecord_clicked();
	//void on_pushButtonPlay_clicked();
	void on_pushButtonAbout_clicked();
	//void on_pushButtonHelp_clicked();
	void on_pushButtonSwitch_clicked();
	void on_pushButtonResetModel_clicked();
	void on_pushButtonResetFrames_clicked();

	void on_checkBoxEuler_stateChanged(int state);

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

private:
	Ui::WindowReceiverDesign ui;
	QNode *qnode;
	//About *about;

	QCheckBox* frame_select_checkbox[NUMBER_OF_FRAMES];
	QComboBox* frame_switch_combobox[NUMBER_OF_FRAMES];
	QLineEdit* frame_address_line[NUMBER_OF_FRAMES];
	QLineEdit* frame_hertz_line[NUMBER_OF_FRAMES];
	QLineEdit* frame_inactivity_line[NUMBER_OF_FRAMES];
};

#endif
