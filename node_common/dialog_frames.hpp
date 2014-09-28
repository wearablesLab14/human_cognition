#ifndef DIALOG_FRAMES_HPP
#define DIALOG_FRAMES_HPP

//QT includes
#include <QDialog>
#include <QGraphicsScene>
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <QImage>

namespace Ui {
class DialogFrames;
}

/*! \brief GUI class for dialog frames window
 * @author Christian Benz <zneb_naitsirhc@web.de>
 * @author Christoph Döringer <christoph.doeringer@gmail.com>
 * @author Hendrik Pfeifer <hendrikpfeifer@gmail.com>
 * @author Heiko Reinemuth <heiko.reinemuth@gmail.com>
 */
class DialogFrames: public QDialog {
Q_OBJECT

public:
	explicit DialogFrames(QWidget *parent = 0);
	~DialogFrames();

private:
	//contains window design and GUI elements of dialog frames window
	Ui::DialogFrames *ui;
};

#endif // DIALOG_FRAMES_HPP
