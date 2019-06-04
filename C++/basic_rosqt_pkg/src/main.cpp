
/*****************************************************************************
** Includes
*****************************************************************************/


#include <QApplication>
#include "../include/gacui/mainwindow.h"
#include "../include/gacui/loginwindow.h"

int main(int argc, char** argv)
{
    QApplication a(argc, argv);
    MainWindow w(argc, argv);
    LoginWindow login;

    login.show();

    // connect signals with slots.
    a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));
    QObject::connect(&login, SIGNAL(showMainWindow()), &w, SLOT(receiveLogin()));
    QObject::connect(&w, SIGNAL(showLoginWindow()), &login, SLOT(receiveLogout()));

    return a.exec();
}
