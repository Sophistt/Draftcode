#include "../include/gacui/loginwindow.h"
#include "ui_loginwindow.h"

LoginWindow::LoginWindow(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::LoginWindow)
{
  ui->setupUi(this);
}

LoginWindow::~LoginWindow()
{
  delete ui;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
void LoginWindow::on_pushButton_login_clicked()
{
  hide();
  emit showMainWindow();
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/
void LoginWindow::receiveLogout()
{
  show();
}
