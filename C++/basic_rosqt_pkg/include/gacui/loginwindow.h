#ifndef LOGINWINDOW_H
#define LOGINWINDOW_H

#include <QDialog>

namespace Ui {
class LoginWindow;
}

class LoginWindow : public QDialog
{
  Q_OBJECT

public:
  explicit LoginWindow(QWidget *parent = 0);
  ~LoginWindow();

private:
  Ui::LoginWindow *ui;

signals:
  void showMainWindow();

public slots:
  /******************************************
  ** Manual connections
  *******************************************/
  void receiveLogout();

private slots:
  void on_pushButton_login_clicked();
};

#endif // LOGINWINDOW_H
