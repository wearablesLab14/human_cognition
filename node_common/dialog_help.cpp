#include "dialog_help.hpp"
#include "ui_dialog_help.h"

/*! \brief Constructor of DialogHelp class
 *
 * @param parent Parent for this window
 */
DialogHelp::DialogHelp(QWidget *parent) :
		QDialog(parent), ui(new Ui::DialogHelp) {

	//setup user interface
	ui->setupUi(this);
}

/*! \brief Destructor of DialogHelp class
 *
 */
DialogHelp::~DialogHelp() {
	delete ui;
}
