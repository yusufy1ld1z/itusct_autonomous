#include "../../include/headers/show_items.hpp"
#include "ui_show_items.h"

showItems::showItems(QWidget *parent, User::User *_user, QString _window_name)
    : QDialog(parent)
    , ui(new Ui::showItems)
{
    this->user = _user;
    this->approve_menu = new approveMenu(this, this->user, "Menu Approval");

    connect(approve_menu, &approveMenu::return_userMenu, this, &showItems::return_userMenu); // for return back to user menu

    ui->setupUi(this);
    setWindowTitle(_window_name);
    QPixmap pix(":icons/gae_logo.png");
    this->ui->label_icon->setPixmap(pix.scaled(150, 150, Qt::KeepAspectRatio));
    this->ui->label_icon->setAlignment(Qt::AlignCenter);

    this->showMenuItems();
}

showItems::~showItems()
{
    delete ui;
}

void showItems::on_pushButton_approveMenu_clicked()
{
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("Approve Menu");
    msgBox.setText("Do you want to override the default preferences for your menu items, otherwise the default values will be kept?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::No);

    int emptyCount = 0;
    int result = msgBox.exec();
    if (result == QMessageBox::Yes) {
        this->hide();

        for (int i = 0; i < static_cast<int>(Menu::DishType::NUM_DISH_TYPES); i++) {
            if (this->user->getMenu()->getMenu()[i].empty()) emptyCount++;
        }
        if(emptyCount == 6){
            QMessageBox::warning(this, "User Menu is Empty", "Your menu is empty, so you can not approve your menu!");
            this->show();
        } else {
            qDebug() << "Menu Approval opening";
            this->approve_menu->addDishWidgets();
            this->approve_menu->show();
        }
    } else if (result == QMessageBox::No) {
        qDebug() << "User clicked No button, default values will be kept.";
        // this->hide();
        // emit return_mainMenu();
    }
}

void showItems::on_pushButton_returnMain_clicked()
{
    qDebug() << "Return to Main Menu from showItems";
    this->hide();
    emit return_mainMenu();
}

void showItems::showMenuItems(){
    for (int i = 0; i < static_cast<int>(Menu::DishType::NUM_DISH_TYPES); i++) {
        Menu::DishType type = static_cast<Menu::DishType>(i);
        std::string dish_type;
        switch (type) {
        case Menu::DishType::STARTER:
            ui->listWidget_starter->clear();
            dish_type = "Starter";
            if(this->user->getMenu()->getMenu()[i].empty()){
                ui->listWidget_starter->addItem(QString::fromStdString("Empty Preference for: " + dish_type));

            } else {
                ui->listWidget_starter->addItem(QString::fromStdString("Your Preferences for: " + dish_type));
                for (auto &item: this->user->getMenu()->getMenu()[i]) {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << (item)->getPrice(); // Set precision to 2 decimal places
                    std::string price_str = ss.str();

                    std::string item_str = item->getName() + "\u200B" + std::string(nameWidth - item->getName().length(), ' ');
                    item_str += "$" + price_str;
                    item_str += '\n';
                    item_str += "---Taste Balance---\n";
                    item_str += "Sweet: " + std::to_string((item)->getTasteBalance().sweet);
                    item_str += " Sour: " + std::to_string((item)->getTasteBalance().sour);
                    item_str += " Salty: " + std::to_string((item)->getTasteBalance().salty);
                    item_str += " Bitter: " + std::to_string((item)->getTasteBalance().bitter);
                    item_str += " Savory: " + std::to_string((item)->getTasteBalance().savory);
                    if(std::shared_ptr<Menu::Starter> starterPtr = std::dynamic_pointer_cast<Menu::Starter>(item)) {
                        item_str += "\nHotness Preference: " + std::string(starterPtr->getIsHot() ? "Hot" : "Cold");
                    }
                    ui->listWidget_starter->addItem(QString::fromStdString(item_str));
                }
            }
            break;

        case Menu::DishType::SALAD:
            ui->listWidget_salad->clear();
            dish_type = "Salad";
            if(this->user->getMenu()->getMenu()[i].empty()){
                ui->listWidget_salad->addItem(QString::fromStdString("Empty Preference for: " + dish_type));

            } else {
                ui->listWidget_salad->addItem(QString::fromStdString("Your Preferences for: " + dish_type));
                for (auto &item: this->user->getMenu()->getMenu()[i]) {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << (item)->getPrice(); // Set precision to 2 decimal places
                    std::string price_str = ss.str();

                    std::string item_str = item->getName() + "\u200B" + std::string(nameWidth - item->getName().length(), ' ');
                    item_str += "$" + price_str;
                    item_str += '\n';
                    item_str += "---Taste Balance---\n";
                    item_str += "Sweet: " + std::to_string((item)->getTasteBalance().sweet);
                    item_str += " Sour: " + std::to_string((item)->getTasteBalance().sour);
                    item_str += " Salty: " + std::to_string((item)->getTasteBalance().salty);
                    item_str += " Bitter: " + std::to_string((item)->getTasteBalance().bitter);
                    item_str += " Savory: " + std::to_string((item)->getTasteBalance().savory);
                    if(std::shared_ptr<Menu::Salads> saladPtr = std::dynamic_pointer_cast<Menu::Salads>(item)) {
                        item_str += "\nAdditional Topping: " + std::string(saladPtr->getAddTopping() ? saladPtr->getTopping() : "No Topping Selected");
                    }

                    ui->listWidget_salad->addItem(QString::fromStdString(item_str));
                }
            }
            break;

        case Menu::DishType::MAIN_COURSE:
            ui->listWidget_mainCourse->clear();
            dish_type = "Main Course";
            if(this->user->getMenu()->getMenu()[i].empty()){
                ui->listWidget_mainCourse->addItem(QString::fromStdString("Empty Preference for: " + dish_type));

            } else {
                ui->listWidget_mainCourse->addItem(QString::fromStdString("Your Preferences for: " + dish_type));
                for (auto &item: this->user->getMenu()->getMenu()[i]) {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << (item)->getPrice(); // Set precision to 2 decimal places
                    std::string price_str = ss.str();

                    std::string item_str = item->getName() + "\u200B" + std::string(nameWidth - item->getName().length(), ' ');
                    item_str += "$" + price_str;
                    item_str += '\n';
                    item_str += "---Taste Balance---\n";
                    item_str += "Sweet: " + std::to_string((item)->getTasteBalance().sweet);
                    item_str += " Sour: " + std::to_string((item)->getTasteBalance().sour);
                    item_str += " Salty: " + std::to_string((item)->getTasteBalance().salty);
                    item_str += " Bitter: " + std::to_string((item)->getTasteBalance().bitter);
                    item_str += " Savory: " + std::to_string((item)->getTasteBalance().savory);
                    if(std::shared_ptr<Menu::MainCourse> mainCoursePtr = std::dynamic_pointer_cast<Menu::MainCourse>(item)) {
                        item_str += "\nVegan: " + std::string(mainCoursePtr->getIsVegan() ? "Yes" : "No");
                    }

                    ui->listWidget_mainCourse->addItem(QString::fromStdString(item_str));
                }
            }
            break;

        case Menu::DishType::DRINK:
            ui->listWidget_drink->clear();
            dish_type = "Drink";
            if(this->user->getMenu()->getMenu()[i].empty()){
                ui->listWidget_drink->addItem(QString::fromStdString("Empty Preference for: " + dish_type));

            } else {
                ui->listWidget_drink->addItem(QString::fromStdString("Your Preferences for: " + dish_type));
                for (auto &item: this->user->getMenu()->getMenu()[i]) {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << (item)->getPrice(); // Set precision to 2 decimal places
                    std::string price_str = ss.str();

                    std::string item_str = item->getName() + "\u200B" + std::string(nameWidth - item->getName().length(), ' ');
                    item_str += "$" + price_str;
                    item_str += '\n';
                    item_str += "---Taste Balance---\n";
                    item_str += "Sweet: " + std::to_string((item)->getTasteBalance().sweet);
                    item_str += " Sour: " + std::to_string((item)->getTasteBalance().sour);
                    item_str += " Salty: " + std::to_string((item)->getTasteBalance().salty);
                    item_str += " Bitter: " + std::to_string((item)->getTasteBalance().bitter);
                    item_str += " Savory: " + std::to_string((item)->getTasteBalance().savory);
                    if(std::shared_ptr<Menu::Drinks> drinkPtr = std::dynamic_pointer_cast<Menu::Drinks>(item)) {
                        item_str += "\nAdditional Carbonation: " + std::string(drinkPtr->getIsCarbonated() ? "Yes" : "No");
                        item_str += "\nAdditional Alcohol: " + std::string(drinkPtr->getIsAlcoholic() ? "Yes" : "No");
                    }

                    ui->listWidget_drink->addItem(QString::fromStdString(item_str));
                }
            }
            break;

        case Menu::DishType::APPETIZER:
            ui->listWidget_appetizer->clear();
            dish_type = "Appetizer";
            if(this->user->getMenu()->getMenu()[i].empty()){
                ui->listWidget_appetizer->addItem(QString::fromStdString("Empty Preference for: " + dish_type));

            } else {
                ui->listWidget_appetizer->addItem(QString::fromStdString("Your Preferences for: " + dish_type));
                for (auto &item: this->user->getMenu()->getMenu()[i]) {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << (item)->getPrice(); // Set precision to 2 decimal places
                    std::string price_str = ss.str();

                    std::string item_str = item->getName() + "\u200B" + std::string(nameWidth - item->getName().length(), ' ');
                    item_str += "$" + price_str;
                    item_str += '\n';
                    item_str += "---Taste Balance---\n";
                    item_str += "Sweet: " + std::to_string((item)->getTasteBalance().sweet);
                    item_str += " Sour: " + std::to_string((item)->getTasteBalance().sour);
                    item_str += " Salty: " + std::to_string((item)->getTasteBalance().salty);
                    item_str += " Bitter: " + std::to_string((item)->getTasteBalance().bitter);
                    item_str += " Savory: " + std::to_string((item)->getTasteBalance().savory);
                    if(std::shared_ptr<Menu::Appetizer> appetizerPtr = std::dynamic_pointer_cast<Menu::Appetizer>(item)) {
                        item_str += "\nService Time Preference: " + std::string(appetizerPtr->getIsBeforeMainCourse() ? "Before the Main Course" : "After the Main Course");
                    }

                    ui->listWidget_appetizer->addItem(QString::fromStdString(item_str));
                }
            }
            break;

        case Menu::DishType::DESSERT:
            ui->listWidget_dessert->clear();
            dish_type = "Dessert";
            if(this->user->getMenu()->getMenu()[i].empty()){
                ui->listWidget_dessert->addItem(QString::fromStdString("Empty Preference for: " + dish_type));

            } else {
                ui->listWidget_dessert->addItem(QString::fromStdString("Your Preferences for: " + dish_type));
                for (auto &item: this->user->getMenu()->getMenu()[i]) {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << (item)->getPrice(); // Set precision to 2 decimal places
                    std::string price_str = ss.str();

                    std::string item_str = item->getName() + "\u200B" + std::string(nameWidth - item->getName().length(), ' ');
                    item_str += "$" + price_str;
                    item_str += '\n';
                    item_str += "---Taste Balance---\n";
                    item_str += "Sweet: " + std::to_string((item)->getTasteBalance().sweet);
                    item_str += " Sour: " + std::to_string((item)->getTasteBalance().sour);
                    item_str += " Salty: " + std::to_string((item)->getTasteBalance().salty);
                    item_str += " Bitter: " + std::to_string((item)->getTasteBalance().bitter);
                    item_str += " Savory: " + std::to_string((item)->getTasteBalance().savory);
                    if(std::shared_ptr<Menu::Desserts> dessertPtr = std::dynamic_pointer_cast<Menu::Desserts>(item)) {
                        item_str += "\nAdditional Chocolate: " + std::string(dessertPtr->getAddChocolate() ? "Yes" : "No");
                    }

                    ui->listWidget_dessert->addItem(QString::fromStdString(item_str));
                }
            }
            break;
        }
    }

    std::string overall_str = this->overallStringGenerate();

    ui->textEdit_menuOverall->setText(QString::fromStdString(overall_str));
}

std::string showItems::overallStringGenerate(){
    std::stringstream ss;
    Menu::TasteBalance taste_bal = this->user->getMenu()->getTasteBalance();
    ss << std::fixed << std::setprecision(2) << this->user->getMenu()->getTotalPrice(); // Set precision to 2 decimal places
    std::string price_str = ss.str();

    std::string overall_str = "Total Price: " + std::string(9, ' ');
    overall_str += "$" + price_str;
    overall_str += '\n';
    overall_str += "---Overall Taste Balance---\n";
    overall_str += "Sweet: " + std::to_string(taste_bal.sweet) + '\n';
    overall_str += "Sour: " + std::to_string(taste_bal.sour) + '\n';
    overall_str += "Salty: " + std::to_string(taste_bal.salty) + '\n';
    overall_str += "Bitter: " + std::to_string(taste_bal.bitter) + '\n';
    overall_str += "Savory: " + std::to_string(taste_bal.savory) + '\n';

    return overall_str;
}


void showItems::on_pushButton_remove_clicked()
{
    std::vector<std::string> selectedItems;

    QList<QListWidget*> listWidgets = {ui->listWidget_starter, ui->listWidget_salad, ui->listWidget_mainCourse, ui->listWidget_drink, ui->listWidget_appetizer, ui->listWidget_dessert};

    for (QListWidget* listWidget : listWidgets) {
        for (auto& item : listWidget->selectedItems()) {
            QString item_text = item->text();
            if (!item_text.contains("Empty Preference for:") && !item_text.contains("Your Preferences for:")) { // Real item, not information widget
                int index = item_text.indexOf(QChar(0x200B)); // Find the position of the first tab character
                if (index != -1) {
                    QString substring = item_text.left(index); // Extract the substring from the beginning to the tab character
                    selectedItems.push_back(substring.toStdString());
                    // qDebug() << substring << " is selected to remove.";

                } else {
                    qDebug() << "No special character found in selected item:" << item_text;
                }
            }
        }
    }

    // Check if the selectedItems vector is empty
    if (selectedItems.empty()) {
        QMessageBox::warning(this, "No Item Selected to Remove", "No item is selected, before removing from your menu, you should choose items first!");
    } else {
        QMessageBox msgBox(this);
        msgBox.setWindowTitle("Validation of Selected Items");
        msgBox.setText("Are you sure to remove these selected items from your menu?");
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::No);

        int result = msgBox.exec();
        if (result == QMessageBox::Yes) {
            qDebug() << "Items to be deleted: ";
            for (const std::string& itemName : selectedItems) {
                qDebug() << QString::fromStdString(itemName);
                emit deleteItemFromUser(itemName);
            }
        } else if (result == QMessageBox::No){
            qDebug() << "User clicked No button for removing";
        }
    }

    std::string overall_str = this->overallStringGenerate();

    this->showMenuItems();
    ui->textEdit_menuOverall->setText(QString::fromStdString(overall_str));
}


