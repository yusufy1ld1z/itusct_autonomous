#ifndef SHOW_ITEMS_HPP
#define SHOW_ITEMS_HPP

#include <QDialog>
#include <QDebug>
#include <QMessageBox>
#include <iomanip>
#include <sstream>
#include "../menu_bot/interface.hpp"
#include "approve_menu.hpp"

namespace Ui {
class showItems;
}

class showItems : public QDialog
{
    Q_OBJECT

const int nameWidth = 40;

public:
    explicit showItems(QWidget *parent = nullptr, User::User *_user = nullptr, QString _window_name = "Dialog");

    void showMenuItems();

    ~showItems();

signals:
    void return_mainMenu();
    void deleteItemFromUser(std::string);

private slots:
    void return_userMenu() {this->showMenuItems(); this->show();};

    void on_pushButton_approveMenu_clicked();

    void on_pushButton_returnMain_clicked();

    void on_pushButton_remove_clicked();

private:
    Ui::showItems *ui;
    User::User *user;
    approveMenu *approve_menu;

    std::string overallStringGenerate();
};

#endif // SHOW_ITEMS_HPP
