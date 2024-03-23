#ifndef SHOW_MENU_HPP
#define SHOW_MENU_HPP

#include <QDialog>
#include <QDebug>
#include <QMessageBox>
#include <iomanip>
#include <sstream>
#include "../menu_bot/interface.hpp"

namespace Ui {
class showMenu;
}

class showMenu : public QDialog
{
    Q_OBJECT

const int nameWidth = 40;

public:
    explicit showMenu(QWidget *parent = nullptr, std::shared_ptr<Menu::Menu> _menu = nullptr, QString _window_name = "Dialog");
    ~showMenu();

signals:
    void return_mainMenu();
    void addItem2User(std::shared_ptr<Menu::MenuItem>);

private slots:
    /**
     * @brief These dish type functions shows the items in available menu of type which it is related.
     */
    void on_pushButton_starter_clicked();

    void on_pushButton_salad_clicked();

    void on_pushButton_main_clicked();

    void on_pushButton_drink_clicked();

    void on_pushButton_appetizer_clicked();

    void on_pushButton_dessert_clicked();

    /**
     * @brief It emits an addition signal with the selected items to add into user menu.
     */
    void on_pushButton_additem_clicked();

    /**
     * @brief Signal emitter to return to main menu.
     */
    void on_pushButton_return_clicked();

private:
    Ui::showMenu *ui;
    std::shared_ptr<Menu::Menu> menu;
    Menu::DishType selected_type;

    /**
     * @brief It iterates over the available menu to find the item by its name.
     * @param _name Name of menu item to be found.
     * @return A shared pointer to the found item by name.
     */
    std::shared_ptr<Menu::MenuItem> findMenuItemByName(std::string _name);
};

#endif // SHOW_MENU_HPP
