#ifndef SUGGESTION_SCREEN_HPP
#define SUGGESTION_SCREEN_HPP

#include <QDialog>
#include <QMessageBox>
#include <QDebug>
#include "../menu_bot/interface.hpp"

namespace Ui {
class suggestionScreen;
}

class suggestionScreen : public QDialog
{
    Q_OBJECT

const int nameWidth = 40;

public:
    explicit suggestionScreen(QWidget *parent = nullptr, User::User * _user = nullptr, std::shared_ptr<Menu::Menu> _available_menu = nullptr, QString _window_name = "Dialog");
    ~suggestionScreen();

signals:
    void return_mainMenu();
    void addItem2User(std::shared_ptr<Menu::MenuItem>);

private slots:
    /**
     * @brief Signal emitter to return to main menu.
     */
    void on_pushButton_return_clicked();

    /**
     * @brief It suggests an item by the user choice after reading the sliders, and add the menu items to list widget.
     */
    void on_pushButton_sugItem_clicked();

    /**
     * @brief It suggests a full item after reading the sliders. It also updates the overall information.
     */
    void on_pushButton_sugMenu_clicked();

    /**
     * @brief It emits an addition signal for each selected items.
     */
    void on_pushButton_add_clicked();

    /**
     * @brief This function is a more convenient way of emitting addition signals for all items in listWidget.
     */
    void on_pushButton_addAll_clicked();

private:
    Ui::suggestionScreen *ui;
    User::User *user;
    std::shared_ptr<Menu::Menu> available_menu;

    /**
     * @brief This function updates the corresponding labels according to the change in horizontal sliders.
     * @param value
     */
    void updateLabelValue(int value);

    /**
     * @brief It iterates over the available menu to find the item by its name.
     * @param _name Name of menu item to be found.
     * @return A shared pointer to the found item by name.
     */
    std::shared_ptr<Menu::MenuItem> findMenuItemByName(std::string _name);
};

#endif // SUGGESTION_SCREEN_HPP
