#include "dialog_about.hpp"
#include "ui_dialog_about.h"

/**
 *
 * @param parent
 */
DialogAbout::DialogAbout(QWidget *parent) :
		QDialog(parent), ui(new Ui::DialogAbout) {
	ui->setupUi(this);
}

/**
 *
 */
DialogAbout::~DialogAbout() {
	delete ui;
}
