#ifndef DIALOG_HELP_HPP
#define DIALOG_HELP_HPP

#include <QDialog>

namespace Ui {
class DialogHelp;
}

/**
 *
 */
class DialogHelp : public QDialog
{
    Q_OBJECT

public:
    explicit DialogHelp(QWidget *parent = 0);
    ~DialogHelp();

private:
    Ui::DialogHelp *ui;
};

#endif // DIALOG_HELP_HPP
