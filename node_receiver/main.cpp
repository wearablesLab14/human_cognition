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
 *		Contains main method of receiver QApplication
 *	-------------------------------------------------------------
 * 	Authors:
 * 		Christian Benz 			<zneb_naitsirhc@web.de>
 * 		Christoph Döringer 		<christoph.doeringer@gmail.com>
 * 		Hendrik Pfeifer 		<hendrikpfeifer@gmail.com>
 * 		Heiko Reinemuth 		<heiko.reinemuth@gmail.com>
 ****************************************************************/

#include <QtGui>
#include <QApplication>

#include "../node_common/window_receiver.hpp"
#include "qnode_receiver.hpp"

/*! \brief Main method of receiver QApplication
 *
 * @param argc Arguments
 * @param argv Arguments
 * @return Exitcode
 */
int main(int argc, char **argv) {

	int exitCode = 0;

	//Receiver QApplication object
	QApplication recvApp(argc, argv);

	//Receiver node object
	QNodeReceiver recvNode(argc, argv);

	//Receiver window object
	WindowReceiver recvWindow(&recvNode);

	//show window
	recvWindow.show();

	//quit application when last window is closed
	recvApp.connect(&recvApp, SIGNAL(lastWindowClosed()), &recvApp,
			SLOT(quit()));

	//while receiver thread is running
	while (recvNode.isRunning()) {

		//process GUI events
		recvApp.processEvents();
	}

	exitCode = recvApp.exec();

	return exitCode;
}
