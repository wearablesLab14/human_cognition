#ifndef human_cognition_WINDOW_LISTENER_H
#define human_cognition_WINDOW_LISTENER_H

//QT includes
#include <QtGui>
#include <QtGui/QMainWindow>

#ifndef Q_MOC_RUN
#include "ui_window_listener.h"
#include "../node_listener/qnode_listener.hpp"
#endif

/*! \brief GUI class for listener node
 * @author Christian Benz <zneb_naitsirhc@web.de>
 * @author Christoph Döringer <christoph.doeringer@gmail.com>
 * @author Hendrik Pfeifer <hendrikpfeifer@gmail.com>
 * @author Heiko Reinemuth <heiko.reinemuth@gmail.com>
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
	void on_checkBoxDisplayCoordinates_stateChanged(int state);
	void on_checkBoxRecordCoordinates_stateChanged(int state);

private:
	/***********************************************
	 PRIVATE GUI METHODS
	 ***********************************************/
	void readSettings();
	void writeSettings();

	//contains window design and GUI elements of listener window
	Ui::WindowListenerDesign uiListener;

	//listener node
	QNodeListener *qnodeListener;
};

#endif
