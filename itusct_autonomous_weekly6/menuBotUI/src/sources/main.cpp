#include "../../include/headers/login_screen.hpp"

#include <QApplication>
#include <QLocale>
#include <QTranslator>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    loginScreen w("Log In");
    w.show();
    return a.exec();
}
