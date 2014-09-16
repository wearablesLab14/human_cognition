#include <QtGui>
#include <QApplication>
#include "../node_common/window_receiver.hpp"
#include "qnode_receiver.hpp"

int main(int argc, char **argv) {

	int exitCode = 0;

    QApplication app(argc, argv);
    QNodeReceiver recvKinePub(argc,argv);
    WindowReceiver w(&recvKinePub);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    exitCode = app.exec();

    return exitCode;
}
