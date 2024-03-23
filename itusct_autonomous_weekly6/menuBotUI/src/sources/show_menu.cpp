#include "../../include/headers/show_menu.hpp"
#include "ui_show_menu.h"

showMenu::showMenu(QWidget *parent, std::shared_ptr<Menu::Menu> _menu, QString _window_name)
    : QDialog(parent)
    , ui(new Ui::showMenu)
{
    this->menu = _menu;
    ui->setupUi(this);
    setWindowTitle(_window_name);
    QPixmap pix(":icons/gae_logo.png");
    this->ui->label_icon->setPixmap(pix.scaled(150, 150, Qt::KeepAspectRatio));
    this->ui->label_icon->setAlignment(Qt::AlignCenter);
}

showMenu::~showMenu()
{
    delete ui;
}

void showMenu::on_pushButton_starter_clicked()
{
    ui->listWidget->clear();
    for (auto it = this->menu->getMenu()[Menu::DishType::STARTER].begin(); it != this->menu->getMenu()[Menu::DishType::STARTER].end(); it++) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << (*it)->getPrice(); // Set precision to 2 decimal places
        std::string price_str = ss.str();

        std::string dish_str = (*it)->getName() + "\u200B" + std::string(nameWidth - (*it)->getName().length(), ' ');
        dish_str += "$" + price_str;
        dish_str += '\n';
        dish_str += "---Taste Balance---\n";
        dish_str += "Sweet: " + std::to_string((*it)->getTasteBalance().sweet);
        dish_str += " Sour: " + std::to_string((*it)->getTasteBalance().sour);
        dish_str += " Salty: " + std::to_string((*it)->getTasteBalance().salty);
        dish_str += " Bitter: " + std::to_string((*it)->getTasteBalance().bitter);
        dish_str += " Savory: " + std::to_string((*it)->getTasteBalance().savory);

        ui->listWidget->addItem(QString::fromStdString(dish_str));
    }
    this->selected_type = Menu::DishType::STARTER;
}


void showMenu::on_pushButton_salad_clicked()
{
    ui->listWidget->clear();
    for (auto it = this->menu->getMenu()[Menu::DishType::SALAD].begin(); it != this->menu->getMenu()[Menu::DishType::SALAD].end(); it++) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << (*it)->getPrice(); // Set precision to 2 decimal places
        std::string price_str = ss.str();

        std::string dish_str = (*it)->getName() + "\u200B" + std::string(nameWidth - (*it)->getName().length(), ' ');
        dish_str += "$" + price_str;
        dish_str += '\n';
        dish_str += "---Taste Balance---\n";
        dish_str += "Sweet: " + std::to_string((*it)->getTasteBalance().sweet);
        dish_str += " Sour: " + std::to_string((*it)->getTasteBalance().sour);
        dish_str += " Salty: " + std::to_string((*it)->getTasteBalance().salty);
        dish_str += " Bitter: " + std::to_string((*it)->getTasteBalance().bitter);
        dish_str += " Savory: " + std::to_string((*it)->getTasteBalance().savory);

        ui->listWidget->addItem(QString::fromStdString(dish_str));
    }
    this->selected_type = Menu::DishType::SALAD;
}


void showMenu::on_pushButton_main_clicked()
{
    ui->listWidget->clear();
    for (auto it = this->menu->getMenu()[Menu::DishType::MAIN_COURSE].begin(); it != this->menu->getMenu()[Menu::DishType::MAIN_COURSE].end(); it++) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << (*it)->getPrice(); // Set precision to 2 decimal places
        std::string price_str = ss.str();

        std::string dish_str = (*it)->getName() + "\u200B" + std::string(nameWidth - (*it)->getName().length(), ' ');
        dish_str += "$" + price_str;
        dish_str += '\n';
        dish_str += "---Taste Balance---\n";
        dish_str += "Sweet: " + std::to_string((*it)->getTasteBalance().sweet);
        dish_str += " Sour: " + std::to_string((*it)->getTasteBalance().sour);
        dish_str += " Salty: " + std::to_string((*it)->getTasteBalance().salty);
        dish_str += " Bitter: " + std::to_string((*it)->getTasteBalance().bitter);
        dish_str += " Savory: " + std::to_string((*it)->getTasteBalance().savory);

        ui->listWidget->addItem(QString::fromStdString(dish_str));
    }
    this->selected_type = Menu::DishType::MAIN_COURSE;
}


void showMenu::on_pushButton_drink_clicked()
{
    ui->listWidget->clear();
    for (auto it = this->menu->getMenu()[Menu::DishType::DRINK].begin(); it != this->menu->getMenu()[Menu::DishType::DRINK].end(); it++) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << (*it)->getPrice(); // Set precision to 2 decimal places
        std::string price_str = ss.str();

        std::string dish_str = (*it)->getName() + "\u200B" + std::string(nameWidth - (*it)->getName().length(), ' ');
        dish_str += "$" + price_str;
        dish_str += '\n';
        dish_str += "---Taste Balance---\n";
        dish_str += "Sweet: " + std::to_string((*it)->getTasteBalance().sweet);
        dish_str += " Sour: " + std::to_string((*it)->getTasteBalance().sour);
        dish_str += " Salty: " + std::to_string((*it)->getTasteBalance().salty);
        dish_str += " Bitter: " + std::to_string((*it)->getTasteBalance().bitter);
        dish_str += " Savory: " + std::to_string((*it)->getTasteBalance().savory);

        ui->listWidget->addItem(QString::fromStdString(dish_str));
    }
    this->selected_type = Menu::DishType::DRINK;
}


void showMenu::on_pushButton_appetizer_clicked()
{
    ui->listWidget->clear();
    for (auto it = this->menu->getMenu()[Menu::DishType::APPETIZER].begin(); it != this->menu->getMenu()[Menu::DishType::APPETIZER].end(); it++) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << (*it)->getPrice(); // Set precision to 2 decimal places
        std::string price_str = ss.str();

        std::string dish_str = (*it)->getName() + "\u200B" + std::string(nameWidth - (*it)->getName().length(), ' ');
        dish_str += "$" + price_str;
        dish_str += '\n';
        dish_str += "---Taste Balance---\n";
        dish_str += "Sweet: " + std::to_string((*it)->getTasteBalance().sweet);
        dish_str += " Sour: " + std::to_string((*it)->getTasteBalance().sour);
        dish_str += " Salty: " + std::to_string((*it)->getTasteBalance().salty);
        dish_str += " Bitter: " + std::to_string((*it)->getTasteBalance().bitter);
        dish_str += " Savory: " + std::to_string((*it)->getTasteBalance().savory);

        ui->listWidget->addItem(QString::fromStdString(dish_str));
    }
    this->selected_type = Menu::DishType::APPETIZER;
}


void showMenu::on_pushButton_dessert_clicked()
{
    ui->listWidget->clear();
    for (auto it = this->menu->getMenu()[Menu::DishType::DESSERT].begin(); it != this->menu->getMenu()[Menu::DishType::DESSERT].end(); it++) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << (*it)->getPrice(); // Set precision to 2 decimal places
        std::string price_str = ss.str();

        std::string dish_str = (*it)->getName() + "\u200B" + std::string(nameWidth - (*it)->getName().length(), ' ');
        dish_str += "$" + price_str;
        dish_str += '\n';
        dish_str += "---Taste Balance---\n";
        dish_str += "Sweet: " + std::to_string((*it)->getTasteBalance().sweet);
        dish_str += " Sour: " + std::to_string((*it)->getTasteBalance().sour);
        dish_str += " Salty: " + std::to_string((*it)->getTasteBalance().salty);
        dish_str += " Bitter: " + std::to_string((*it)->getTasteBalance().bitter);
        dish_str += " Savory: " + std::to_string((*it)->getTasteBalance().savory);

        ui->listWidget->addItem(QString::fromStdString(dish_str));
    }
    this->selected_type = Menu::DishType::DESSERT;
}

void showMenu::on_pushButton_additem_clicked()
{
    QList<QListWidgetItem*> selectedItems = ui->listWidget->selectedItems();

    if (!selectedItems.isEmpty()) {
        QMessageBox msgBox(this);
        msgBox.setWindowTitle("Validation of Selected Items");
        msgBox.setText("Are you sure to add these selected items to your menu?");
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::No);

        int result = msgBox.exec();
        if (result == QMessageBox::Yes) {
            // qDebug() << "User clicked Yes button";

            for (QListWidgetItem* item : selectedItems) {
                QString text = item->text();
                int index = text.indexOf(QChar(0x200B)); // Find the position of the first tab character
                if (index != -1) {
                    QString substring = text.left(index); // Extract the substring from the beginning to the tab character
                    std::shared_ptr<Menu::MenuItem> item2add = this->findMenuItemByName(substring.toStdString());
                    if(item2add){
                        qDebug() << QString::fromStdString(item2add->getName()) << " selected.";
                        emit addItem2User(item2add);
                    }
                } else {
                    qDebug() << "No special character found in selected item:" << text;
                }
            }
            // this->on_pushButton_return_clicked(); // return to main menu
        } else if (result == QMessageBox::No) {
            qDebug() << "User clicked No button";
        }

    } else {
        QMessageBox::warning(this, "No Item Selected", "No item is selected, before adding to your menu, you should choose items first!");
    }
}


void showMenu::on_pushButton_return_clicked()
{
    qDebug() << "Return to Main Menu from showMenu";
    this->hide();
    emit return_mainMenu();
}

std::shared_ptr<Menu::MenuItem> showMenu::findMenuItemByName(std::string _name){
    for (auto& item : this->menu->getMenu()[this->selected_type]){
        if (item->getName() == _name){
            return item;
        }
    }
    return nullptr;
}

