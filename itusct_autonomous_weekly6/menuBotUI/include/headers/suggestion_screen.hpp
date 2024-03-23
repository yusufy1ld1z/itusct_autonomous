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
    void on_pushButton_return_clicked();

    void on_pushButton_sugItem_clicked();

    void on_pushButton_sugMenu_clicked();

    void on_pushButton_add_clicked();

    void on_pushButton_addAll_clicked();

private:
    Ui::suggestionScreen *ui;
    User::User *user;
    std::shared_ptr<Menu::Menu> available_menu;

    void updateLabelValue(int value);
    std::shared_ptr<Menu::MenuItem> findMenuItemByName(std::string _name);
};

#endif // SUGGESTION_SCREEN_HPP
