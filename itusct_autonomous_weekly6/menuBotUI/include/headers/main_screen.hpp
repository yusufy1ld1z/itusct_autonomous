#ifndef MAIN_SCREEN_HPP
#define MAIN_SCREEN_HPP

#include <QDialog>
#include <QMessageBox>
#include "../menu_bot/interface.hpp"
#include "show_menu.hpp"
#include "show_items.hpp"
#include "suggestion_screen.hpp"

namespace Ui {
class mainScreen;
}

class mainScreen : public QDialog
{
    Q_OBJECT

public:
    explicit mainScreen(QWidget *parent = nullptr, User::User *_user = nullptr, QString _window_name = "Dialog");
    ~mainScreen();

private slots:

    void return_mainMenu() {this->show();};

    void addItem2User(std::shared_ptr<Menu::MenuItem>);

    void deleteItemFromUser(std::string);

    void on_pushButton_seeMenu_clicked();

    void on_pushButton_seeItems_clicked();

    void on_pushButton_sugBot_clicked();

    void on_pushButton_exit_clicked();

private:
    User::User *user;
    std::shared_ptr<Menu::Menu> available_menu;
    Ui::mainScreen *ui;
    showMenu *show_menu;
    showItems *show_items;
    suggestionScreen *suggestion;
};

#endif // MAIN_SCREEN_HPP
