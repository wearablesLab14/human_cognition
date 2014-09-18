#ifndef human_cognition_WINDOW_LISTENER_H
#define human_cognition_WINDOW_LISTENER_H

#include <QtGui>
#include <QtGui/QMainWindow>

#ifndef Q_MOC_RUN
#include "ui_window_listener.h"
#include "../node_listener/qnode_listener.hpp"
#endif

/**
 *
 */
class WindowListener: public QMainWindow {
Q_OBJECT

public:
	WindowListener(QNodeListener *node, QWidget *parent = 0);
	~WindowListener();
	void closeEvent(QCloseEvent *event);

public Q_SLOTS:
	/***********************************************
	 MODEL SIGNAL METHODS
	 ***********************************************/
	void updateListView();

	/***********************************************
	 GUI ACTION METHODS
	 ***********************************************/
	void on_pushButtonListenerSetup_clicked();
	void on_pushButtonListenerStart_clicked();
	void on_pushButtonListenerStop_clicked();
	void on_checkBoxRecordCoordinates_stateChanged(int state);
	void on_checkBoxDisplayCoordinates_stateChanged(int state);

private:
	/***********************************************
	 GUI SETTINGS
	 ***********************************************/
	void readSettings();
	void writeSettings();

	Ui::WindowListenerDesign ui_list;
	QNodeListener *qnode_list;
};

#endif
