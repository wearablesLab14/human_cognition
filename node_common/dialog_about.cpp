#include "dialog_about.hpp"
#include "ui_dialog_about.h"

/*! \brief Constructor of DialogAbout class
 *
 * @param parent Parent for this window
 */
DialogAbout::DialogAbout(QWidget *parent) :
		QDialog(parent), ui(new Ui::DialogAbout) {

	//setup user interface
	ui->setupUi(this);
}

/*! \brief Destructor of DialogAbout class
 *
 */
DialogAbout::~DialogAbout() {
	delete ui;
}
