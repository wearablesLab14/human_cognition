/****************************************************************
 *  Project:
 *  	Integrating Body- and Eye-Tracking to study Cognition in the Wild
 *	-------------------------------------------------------------
 * 	TU Darmstadt
 * 	Department Computer Science
 * 	Summer term 2014
 *	-------------------------------------------------------------
 *	File: main.cpp
 *	Description:
 *		Contains main method of listener QApplication
 *	-------------------------------------------------------------
 * 	Authors:
 * 		Christian Benz 			<zneb_naitsirhc@web.de>
 * 		Christoph Döringer 		<christoph.doeringer@gmail.com>
 * 		Hendrik Pfeifer 		<hendrikpfeifer@gmail.com>
 * 		Heiko Reinemuth 		<heiko.reinemuth@gmail.com>
 ****************************************************************/

//QT includes
#include <QtGui>
#include <QApplication>

#include "../node_common/window_listener.hpp"
#include "qnode_listener.hpp"

/*! \brief Main method of listener QApplication
 *
 * @param argc Arguments
 * @param argv Arguments
 * @return Exitcode
 */
int main(int argc, char **argv) {

	int exitCode = 0;

	//Listener QApplication object
	QApplication listenerApp(argc, argv);

	//Receiver node object
	QNodeListener listenerNode(argc, argv);

	//Receiver window object
	WindowListener listenerWindow(&listenerNode);

	//show window
	listenerWindow.show();

	//quit application when last window is closed
	listenerApp.connect(&listenerApp, SIGNAL(lastWindowClosed()), &listenerApp,
			SLOT(quit()));

	//while receiver thread is running
	while (listenerNode.isRunning()) {

		//process GUI events
		listenerApp.processEvents();
	}
	exitCode = listenerApp.exec();

	return exitCode;
}
