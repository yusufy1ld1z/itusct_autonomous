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
    /**
     * @brief Slot for returning to main menu.
     */
    void return_mainMenu() {this->show();};

    /**
     * @brief It adds item to the user menu, menu item is taken from the signals of other screens.
     * @param _item A shared pointer to the item to be added.
     */
    void addItem2User(std::shared_ptr<Menu::MenuItem> _item);

    /**
     * @brief It removes item from the user menu, menu item's name is taken from the signals of other screens.
     * @param _name A name of the item to be removed.
     */
    void deleteItemFromUser(std::string _name);

    /**
     * @brief It opens up the show_menu window.
     */
    void on_pushButton_seeMenu_clicked();

    /**
     * @brief It opens up the show_items window.
     */
    void on_pushButton_seeItems_clicked();

    /**
     * @brief It opens up the suggestion window.
     */
    void on_pushButton_sugBot_clicked();

    /**
     * @brief It terminates the program after a validation from user.
     */
    void on_pushButton_exit_clicked();

private:
    User::User *user;
    std::shared_ptr<Menu::Menu> available_menu;
    Ui::mainScreen *ui;

    // sub window pointers
    showMenu *show_menu;
    showItems *show_items;
    suggestionScreen *suggestion;
};

#endif // MAIN_SCREEN_HPP
