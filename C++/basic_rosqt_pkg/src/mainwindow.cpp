#include "../include/gacui/mainwindow.h"
#include "ui_mainwindow.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
  QMainWindow(parent),
  pubnode(argc, argv),
  subnode(argc, argv),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  QObject::connect(&pubnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
  ** Logging
  **********************/
  ui->view_logging->setModel(pubnode.loggingModel());
  QObject::connect(&pubnode, SIGNAL(loggingUpdated()), this, SLOT(updatePubLogView()));

  ui->view_subscribe->setModel(subnode.loggingModel());
  QObject::connect(&subnode, SIGNAL(loggingUpdated()), this, SLOT(updatePubLogView()));
}

MainWindow::~MainWindow()
{
  delete ui;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
void MainWindow::on_pushButton_logout_clicked()
{
    hide();
    emit showLoginWindow();
}

void MainWindow::on_pushButton_publish_clicked()
{
  if ( !pubnode.init() ) {
      showNoMasterMessage();
  } else {
      // Set button_publish unclickable.
      ui->pushButton_publish->setEnabled(false);
  }
}

void MainWindow::on_pushButton_subscribe_clicked()
{
  if ( !subnode.init() ) {
      showNoMasterMessage();
  } else {
      // Set pushButton_subscribe unclickable.
      ui->pushButton_subscribe->setEnabled(false);
  }
}


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/
void MainWindow::showNoMasterMessage()
{
    QMessageBox::critical(this, "Error", "Couldn't find the ros master.");
}


void MainWindow::receiveLogin()
{
  show();
}

void MainWindow::updatePubLogView()
{
  ui->view_logging->scrollToBottom();
}

void MainWindow::updateSubLogView()
{
  ui->view_subscribe->scrollToBottom();
}


