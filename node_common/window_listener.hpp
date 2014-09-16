#ifndef human_cognition_WINDOW_LISTENER_H
#define human_cognition_WINDOW_LISTENER_H

#include <QtGui>
#include <QtGui/QMainWindow>

#ifndef Q_MOC_RUN
#include "node_common/ui_window_listener.h"
#include "qnode.hpp"
#endif

class WindowListener: public QMainWindow {
Q_OBJECT

public:
	WindowListener(QNode *node, QWidget *parent = 0);
	~WindowListener();
	void closeEvent(QCloseEvent *event);

	void readSettings();
	void writeSettings();

public Q_SLOTS:
	void updateListInfo();

	void on_checkBoxRecordCoordinates_stateChanged(int state);
	void on_checkBoxDisplayCoordinates_stateChanged(int state);
	void on_pushButtonListenerSetup_clicked();
	void on_pushButtonListenerStart_clicked();
	void on_pushButtonListenerStop_clicked();

private:
	Ui::WindowListenerDesign ui;
	QNode *qnode;
};

#endif
