#include "../../include/headers/main_screen.hpp"
#include "ui_main_screen.h"
#include <QDebug>

mainScreen::mainScreen(QWidget *parent, User::User *_user, QString _window_name)
    : QDialog(parent)
    , ui(new Ui::mainScreen)
{
    this->user = _user;
    std::string file_name = "/home/yildiz/Desktop/gae/autonomous_assignments/itusct_autonomous_weekly5/assignment5_solutions/menuBot/menu.json";
    std::vector<std::vector<std::shared_ptr<Menu::MenuItem>>> availableMenu(6);
    try {
        Interface::readFromJSON(file_name, availableMenu);
        qDebug() << "Menu succesfully loaded";
    }
    catch (const std::exception& e) {
        qDebug() << "Cant parse JSON. Error: " << e.what();
        QApplication::quit();
        // return 1;
    }
    this->available_menu = std::make_shared<Menu::Menu>(availableMenu);

    this->show_menu = new showMenu(this, this->available_menu, "Available Menu");
    this->show_items = new showItems(this, this->user, "User Menu");
    this->suggestion = new suggestionScreen(this, this->user, this->available_menu, "Suggestion Bot");

    connect(show_menu, &showMenu::return_mainMenu, this, &mainScreen::return_mainMenu); // for return back
    connect(show_menu, &showMenu::addItem2User, this, &mainScreen::addItem2User); // for add item
    connect(show_items, &showItems::return_mainMenu, this, &mainScreen::return_mainMenu); // for return back
    connect(show_items, &showItems::deleteItemFromUser, this, &mainScreen::deleteItemFromUser); // for delete item
    connect(suggestion, &suggestionScreen::return_mainMenu, this, &mainScreen::return_mainMenu); // for return back
    connect(suggestion, &suggestionScreen::addItem2User, this, &mainScreen::addItem2User); // for add item

    ui->setupUi(this);
    setWindowTitle(_window_name);
    QPixmap pix(":icons/gae_logo.png");
    this->ui->label_icon->setPixmap(pix.scaled(150, 150, Qt::KeepAspectRatio));
    this->ui->label_icon->setAlignment(Qt::AlignCenter);
}

mainScreen::~mainScreen()
{
    delete ui;
}


void mainScreen::on_pushButton_seeMenu_clicked()
{
    qDebug() << "Available Menu opening";
    this->hide();

    this->show_menu->show();
}


void mainScreen::on_pushButton_seeItems_clicked()
{
    qDebug() << "User Menu opening";
    this->hide();

    this->show_items->showMenuItems();
    this->show_items->show();
}

void mainScreen::on_pushButton_exit_clicked()
{
    QString gender_str = (this->user->getGender() == User::Gender::MR ? "Mr. " : "Mrs. ");
    // qDebug() << "Gender string:" << gender_str;
    // qDebug() << "First name:" << QString::fromStdString(this->user->getFirstName());
    // qDebug() << "Last name:" << QString::fromStdString(this->user->getLastName());
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("Good Bye!");
    msgBox.setText("Good bye " + gender_str +
                   QString::fromStdString(this->user->getFirstName()) + " " + QString::fromStdString(this->user->getLastName()) + ", see you next time!");
    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Ok);

    int result = msgBox.exec();

    if (result == QMessageBox::Ok) {
        // qDebug() << "User clicked Ok button";
        QApplication::quit();
    } else if (result == QMessageBox::Cancel) {
        // qDebug() << "User clicked Cancel button";
        msgBox.close();
    }

}

void mainScreen::addItem2User(std::shared_ptr<Menu::MenuItem> _item){
    this->user->getMenu()->addMenuItem(_item);
    qDebug() << QString::fromStdString(_item->getName()) << " is added successfully.";
}

void mainScreen::deleteItemFromUser(std::string _name){
    this->user->getMenu()->removeMenuItem(_name);
    qDebug() << QString::fromStdString(_name) << " is removed successfully.";
}

void mainScreen::on_pushButton_sugBot_clicked()
{
    qDebug() << "Suggestion Bot opening";
    this->hide();

    this->suggestion->show();
}

