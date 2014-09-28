#include "dialog_frames.hpp"
#include "ui_dialog_frames.h"

/*! \brief Constructor of DialogFrames class
 *
 * @param parent Parent for this window
 */
DialogFrames::DialogFrames(QWidget *parent) :
		QDialog(parent), ui(new Ui::DialogFrames) {

	//setup user interface
	ui->setupUi(this);

	//add image to graphicsView
	QGraphicsScene* scene = new QGraphicsScene();
	QGraphicsPixmapItem* item = new QGraphicsPixmapItem(
			QPixmap::fromImage(QImage("://images/frames.png")));
	scene->addItem(item);
	ui->graphicsView->setScene(scene);
	ui->graphicsView->show();
}

/*! \brief Destructor of DialogFrames class
 *
 */
DialogFrames::~DialogFrames() {
	delete ui;
}
