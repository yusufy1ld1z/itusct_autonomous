#ifndef LOGIN_SCREEN_H
#define LOGIN_SCREEN_H

#include <QApplication>
#include <QDialog>
#include <QMessageBox>
#include "main_screen.hpp"
#include "../menu_bot/interface.hpp"

namespace Ui {
class loginScreen;
}

class loginScreen : public QDialog
{
    Q_OBJECT

public:
    explicit loginScreen(QString _window_name = "Dialog", QWidget *parent = nullptr);
    ~loginScreen();

private slots:
    /**
     * @brief It fills up the user informations after validation of inputs, then enters the main_screen window.
     */
    void on_pushButton_enter_clicked();

    /**
     * @brief It says goodbye to the user.
     */
    void on_pushButton_quit_clicked();

private:
    User::User *user;
    Ui::loginScreen *ui;
    mainScreen *main_screen;
};

#endif // LOGIN_SCREEN_H
