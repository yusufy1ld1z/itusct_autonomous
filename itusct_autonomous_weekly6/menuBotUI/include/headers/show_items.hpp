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

    /**
     * @brief It creates, and adds listWidget items for each dish type including specific additional preferences, price, taste balance, and name.
     */
    void showMenuItems();

    ~showItems();

signals:
    void return_mainMenu();
    void deleteItemFromUser(std::string);

private slots:
    void return_userMenu() {this->showMenuItems(); this->show();};

    /**
     * @brief It first controls whether the user menu is empty or not, then opens then shows the approve_menu window. It generates the widgets before showing the window.
     */
    void on_pushButton_approveMenu_clicked();

    /**
     * @brief Signal emitter to return to main menu.
     */
    void on_pushButton_returnMain_clicked();

    /**
     * @brief It first controls whether the user menu is empty or not, then emits a signal to remove selected items.
     */
    void on_pushButton_remove_clicked();

private:
    Ui::showItems *ui;
    User::User *user;
    approveMenu *approve_menu;

    /**
    * @brief It generates the string containing overall taste balance and total price of the user menu.
    * @return A string to be shown in user menu's overall part.
    */
    std::string overallStringGenerate();
};

#endif // SHOW_ITEMS_HPP
