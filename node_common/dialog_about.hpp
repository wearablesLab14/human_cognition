#ifndef DIALOG_ABOUT_HPP
#define DIALOG_ABOUT_HPP

//QT includes
#include <QDialog>

namespace Ui {
class DialogAbout;
}

/*! \brief GUI class for dialog about window
 * @author Christian Benz <zneb_naitsirhc@web.de>
 * @author Christoph Döringer <christoph.doeringer@gmail.com>
 * @author Hendrik Pfeifer <hendrikpfeifer@gmail.com>
 * @author Heiko Reinemuth <heiko.reinemuth@gmail.com>
 */
class DialogAbout: public QDialog {
Q_OBJECT

public:
	explicit DialogAbout(QWidget *parent = 0);
	~DialogAbout();

private:
	//contains window design and GUI elements of dialog about window
	Ui::DialogAbout *ui;
};

#endif // DIALOG_ABOUT_HPP
