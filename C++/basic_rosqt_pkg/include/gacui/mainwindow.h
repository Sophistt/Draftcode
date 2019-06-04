#ifndef MAINWINDOW_H
#define MAINWINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <QMainWindow>
#include <QListView>
#include <QMessageBox>
#include "pubnode.h"
#include "subnode.h"


/*****************************************************************************
** Namespace
*****************************************************************************/
namespace Ui {
class MainWindow;
}

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void showNoMasterMessage();


public slots:
  /******************************************
  ** Manual connections
  *******************************************/
  void receiveLogin();
  void updatePubLogView();
  void updateSubLogView();

private slots:
  void on_pushButton_logout_clicked();

  void on_pushButton_publish_clicked();

  void on_pushButton_subscribe_clicked();

signals:
  void showLoginWindow();

private:
  Ui::MainWindow *ui;
  PubNode pubnode;
  SubNode subnode;

};

#endif // MAINWINDOW_H
