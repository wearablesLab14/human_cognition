#include "dialog_frames.hpp"
#include "ui_dialog_frames.h"

DialogFrames::DialogFrames(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogFrames)
{
    ui->setupUi(this);

    QGraphicsScene* scene = new QGraphicsScene();
	QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(QImage("://images/frames.png")));
	scene->addItem(item);
	ui->graphicsView->setScene(scene);
	ui->graphicsView->show();
}

DialogFrames::~DialogFrames()
{
    delete ui;
}
