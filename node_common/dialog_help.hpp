#ifndef DIALOG_HELP_HPP
#define DIALOG_HELP_HPP

//QT includes
#include <QDialog>

namespace Ui {
class DialogHelp;
}

/*! \brief GUI class for dialog help window
 * @author Christian Benz <zneb_naitsirhc@web.de>
 * @author Christoph Döringer <christoph.doeringer@gmail.com>
 * @author Hendrik Pfeifer <hendrikpfeifer@gmail.com>
 * @author Heiko Reinemuth <heiko.reinemuth@gmail.com>
 */
class DialogHelp: public QDialog {
Q_OBJECT

public:
	explicit DialogHelp(QWidget *parent = 0);
	~DialogHelp();

private:
	//contains window design and GUI elements of dialog help window
	Ui::DialogHelp *ui;
};

#endif // DIALOG_HELP_HPP
