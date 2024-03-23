#include "../../include/headers/login_screen.hpp"
#include "ui_login_screen.h"

loginScreen::loginScreen(QString _window_name, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::loginScreen)
{
    this->user = new User::User();
    ui->setupUi(this);
    setWindowTitle(_window_name);
}

loginScreen::~loginScreen()
{
    delete ui;
}

void loginScreen::on_pushButton_enter_clicked()
{
    QString user_name = ui->lineEdit_name->text();
    QString user_surname = ui->lineEdit_surname->text();
    User::Gender user_gender = User::Gender::MR;
    bool login_validate = true;

    if (user_name.isEmpty() || user_surname.isEmpty())
        login_validate = false;

    if (ui->radioButton_mr->isChecked())
        user_gender = User::Gender::MR;
    else if (ui->radioButton_mrs->isChecked())
        user_gender = User::Gender::MRS;
    else login_validate = false;

    this->user->setFirstName(user_name.toStdString());
    this->user->setLastName(user_surname.toStdString());
    this->user->setGender(user_gender);

    if (login_validate){
        QMessageBox::information(this, "Welcome!", "Welcome to ITU GAE Restaurant, " + user_name + " " + user_surname + "!");
        this->hide();
        this->main_screen = new mainScreen(this, this->user, "Main Menu");
        main_screen->show();
    } else {
        QMessageBox::warning(this, "Login Error", "Incorrect login information, check whether all required fields are filled correctly!");
    }
}


void loginScreen::on_pushButton_quit_clicked()
{
    QMessageBox::information(this, "Good Bye!", "Good bye, see you next time!");
    QApplication::quit();
}

