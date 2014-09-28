#include "dialog_help.hpp"
#include "ui_dialog_help.h"

DialogHelp::DialogHelp(QWidget *parent) :
		QDialog(parent), ui(new Ui::DialogHelp) {
	ui->setupUi(this);
}

DialogHelp::~DialogHelp() {
	delete ui;
}
