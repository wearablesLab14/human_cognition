#include <QtGui>
#include <QApplication>

#include "../node_common/window_listener.hpp"
#include "qnode_listener.hpp"

int main(int argc, char **argv) {

	int exitCode = 0;

	QApplication app(argc, argv);
	QNodeListener listener(argc, argv);
	WindowListener w(&listener);
	w.show();
	app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
	exitCode = app.exec();

	return exitCode;
}
