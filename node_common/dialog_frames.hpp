#ifndef DIALOG_FRAMES_HPP
#define DIALOG_FRAMES_HPP

#include <QDialog>
#include <QGraphicsScene>
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <QImage>

namespace Ui {
class DialogFrames;
}

class DialogFrames : public QDialog
{
    Q_OBJECT

public:
    explicit DialogFrames(QWidget *parent = 0);
    ~DialogFrames();

private:
    Ui::DialogFrames *ui;
};

#endif // DIALOG_FRAMES_HPP
