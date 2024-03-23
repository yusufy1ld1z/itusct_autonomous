#include "../../include/headers/suggestion_screen.hpp"
#include "ui_suggestion_screen.h"

suggestionScreen::suggestionScreen(QWidget *parent, User::User *_user, std::shared_ptr<Menu::Menu> _available_menu, QString _window_name)
    : QDialog(parent)
    , ui(new Ui::suggestionScreen)
{
    this->user = _user;
    this->available_menu = _available_menu;

    ui->setupUi(this);
    setWindowTitle(_window_name);
    QPixmap pix(":icons/gae_logo.png");
    this->ui->label_icon->setPixmap(pix.scaled(150, 150, Qt::KeepAspectRatio));
    this->ui->label_icon->setAlignment(Qt::AlignCenter);

    connect(ui->horizontalSlider_sweet, &QSlider::valueChanged, this, &suggestionScreen::updateLabelValue);
    connect(ui->horizontalSlider_sour, &QSlider::valueChanged, this, &suggestionScreen::updateLabelValue);
    connect(ui->horizontalSlider_bitter, &QSlider::valueChanged, this, &suggestionScreen::updateLabelValue);
    connect(ui->horizontalSlider_salty, &QSlider::valueChanged, this, &suggestionScreen::updateLabelValue);
    connect(ui->horizontalSlider_savory, &QSlider::valueChanged, this, &suggestionScreen::updateLabelValue);
}

suggestionScreen::~suggestionScreen()
{
    delete ui;
}

void suggestionScreen::on_pushButton_return_clicked()
{
    qDebug() << "Return to Main Menu from suggestionScreen";
    this->hide();
    emit return_mainMenu();
}

void suggestionScreen::updateLabelValue(int value) {
    // Determine which slider emitted the signal
    QSlider* senderSlider = qobject_cast<QSlider*>(sender());
    if (!senderSlider) {
        return; // Safety check
    }

    // Get the corresponding label
    QLabel* correspondingLabel = nullptr;
    if (senderSlider == ui->horizontalSlider_sweet) {
        correspondingLabel = ui->label_sweet;
    } else if (senderSlider == ui->horizontalSlider_sour) {
        correspondingLabel = ui->label_sour;
    } else if (senderSlider == ui->horizontalSlider_bitter) {
        correspondingLabel = ui->label_bitter;
    } else if (senderSlider == ui->horizontalSlider_salty) {
        correspondingLabel = ui->label_salty;
    } else if (senderSlider == ui->horizontalSlider_savory) {
        correspondingLabel = ui->label_savory;
    }

    // Update the label text with the slider value
    if (correspondingLabel) {
        correspondingLabel->setText(QString::number(value));
    }
}

void suggestionScreen::on_pushButton_sugItem_clicked()
{
    ui->listWidget->clear();
    bool radio_validator = true;
    std::string dish_type_str = "";
    Menu::DishType dish_type = Menu::DishType::STARTER;
    if (ui->radioButton_starter->isChecked()) {dish_type = Menu::DishType::STARTER; dish_type_str = "Starter";}
    else if (ui->radioButton_salad->isChecked()) {dish_type = Menu::DishType::SALAD; dish_type_str = "Salad";}
    else if (ui->radioButton_mainCourse->isChecked()) {dish_type = Menu::DishType::MAIN_COURSE; dish_type_str = "Main Course";}
    else if (ui->radioButton_drink->isChecked()) {dish_type = Menu::DishType::DRINK; dish_type_str = "Drink";}
    else if (ui->radioButton_appetizer->isChecked()) {dish_type = Menu::DishType::APPETIZER; dish_type_str = "Appetizer";}
    else if (ui->radioButton_dessert->isChecked()) {dish_type = Menu::DishType::DESSERT; dish_type_str = "Dessert";}
    else radio_validator = false;

    Menu::TasteBalance taste_balance;
    taste_balance.sweet = std::stoi(ui->label_sweet->text().toStdString());
    taste_balance.sour = std::stoi(ui->label_sour->text().toStdString());
    taste_balance.bitter = std::stoi(ui->label_bitter->text().toStdString());
    taste_balance.salty = std::stoi(ui->label_salty->text().toStdString());
    taste_balance.savory = std::stoi(ui->label_savory->text().toStdString());

    if (radio_validator) {
        std::shared_ptr<Menu::MenuItem> suggested_menu_item = Interface::suggestMenuItem(this->available_menu, taste_balance, dish_type);
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << suggested_menu_item->getPrice(); // Set precision to 2 decimal places
        std::string price_str = ss.str();

        std::string dish_str = dish_type_str;
        dish_str += "\n-------------------\n";

        dish_str += suggested_menu_item->getName() + "\u200B" + std::string(nameWidth - suggested_menu_item->getName().length(), ' ');
        dish_str += "$" + price_str;
        dish_str += '\n';
        dish_str += "---Taste Balance---\n";
        dish_str += "Sweet: " + std::to_string(suggested_menu_item->getTasteBalance().sweet);
        dish_str += " Sour: " + std::to_string(suggested_menu_item->getTasteBalance().sour);
        dish_str += " Salty: " + std::to_string(suggested_menu_item->getTasteBalance().salty);
        dish_str += " Bitter: " + std::to_string(suggested_menu_item->getTasteBalance().bitter);
        dish_str += " Savory: " + std::to_string(suggested_menu_item->getTasteBalance().savory);

        ui->listWidget->addItem(QString::fromStdString(dish_str));

    } else {
        QMessageBox::warning(this, "Selection Error", "Dish type must be selected, check whether all required fields are filled correctly!");
    }

    ui->textEdit_menuOverall->clear();
}


void suggestionScreen::on_pushButton_sugMenu_clicked()
{
    // menu suggestion, read the sliders, and add the menu items to list widget
    ui->listWidget->clear();
    Menu::TasteBalance taste_balance;
    taste_balance.sweet = std::stoi(ui->label_sweet->text().toStdString());
    taste_balance.sour = std::stoi(ui->label_sour->text().toStdString());
    taste_balance.bitter = std::stoi(ui->label_bitter->text().toStdString());
    taste_balance.salty = std::stoi(ui->label_salty->text().toStdString());
    taste_balance.savory = std::stoi(ui->label_savory->text().toStdString());

    std::shared_ptr<Menu::Menu> suggested_menu = Interface::suggestFullMenu(this->available_menu, taste_balance);
    for (int i = 0; i < static_cast<int>(Menu::DishType::NUM_DISH_TYPES); i++) {
        Menu::DishType type = static_cast<Menu::DishType>(i);
        std::string dish_type = "";
        std::string dish_str;
        switch (type) {
        case Menu::DishType::STARTER:
            dish_type = "Starter";
            break;
        case Menu::DishType::SALAD:
            dish_type = "Salad";
            break;
        case Menu::DishType::MAIN_COURSE:
            dish_type = "Main Course";
            break;
        case Menu::DishType::DRINK:
            dish_type = "Drink";
            break;
        case Menu::DishType::APPETIZER:
            dish_type = "Appetizer";
            break;
        case Menu::DishType::DESSERT:
            dish_type = "Dessert";
            break;
        }

        dish_str = dish_type;
        dish_str += "\n-------------------\n";

        for (auto &item: suggested_menu->getMenu()[i]) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << item->getPrice(); // Set precision to 2 decimal places
            std::string price_str = ss.str();

            dish_str += item->getName() + "\u200B" + std::string(nameWidth - item->getName().length(), ' ');
            dish_str += "$" + price_str;
            dish_str += '\n';
            dish_str += "---Taste Balance---\n";
            dish_str += "Sweet: " + std::to_string(item->getTasteBalance().sweet);
            dish_str += " Sour: " + std::to_string(item->getTasteBalance().sour);
            dish_str += " Salty: " + std::to_string(item->getTasteBalance().salty);
            dish_str += " Bitter: " + std::to_string(item->getTasteBalance().bitter);
            dish_str += " Savory: " + std::to_string(item->getTasteBalance().savory);

            ui->listWidget->addItem(QString::fromStdString(dish_str));
        }
    }

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << suggested_menu->getTotalPrice(); // Set precision to 2 decimal places
    std::string overall_price_str = ss.str();

    std::string overall_str = "Total Price: " + std::string(9, ' ');
    overall_str += "$" + overall_price_str;
    overall_str += '\n';
    overall_str += "---Overall Taste Balance---\n";
    overall_str += "Sweet: " + std::to_string(suggested_menu->getTasteBalance().sweet) + ' ';
    overall_str += "Sour: " + std::to_string(suggested_menu->getTasteBalance().sour) + ' ';
    overall_str += "Salty: " + std::to_string(suggested_menu->getTasteBalance().salty) + ' ';
    overall_str += "Bitter: " + std::to_string(suggested_menu->getTasteBalance().bitter) + ' ';
    overall_str += "Savory: " + std::to_string(suggested_menu->getTasteBalance().savory);

    ui->textEdit_menuOverall->setText(QString::fromStdString(overall_str));
}


void suggestionScreen::on_pushButton_add_clicked()
{
    QList<QListWidgetItem*> selectedItems = ui->listWidget->selectedItems();

    if (!selectedItems.isEmpty()) {
        // Extract substring until the first tab character
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
                int total_pass_char = 20;
                int startPos = text.indexOf("-");
                if (startPos == -1) {
                    qDebug() << "Could not find the start of the first '-' sequence";
                }
                int first_index = total_pass_char + startPos;
                int last_index = text.indexOf(QChar(0x200B)); // Find the position of the first tab character
                if (last_index != -1) {
                    QString substring = text.mid(first_index, last_index - first_index); // Extract the substring from the beginning to the tab character
                    // qDebug() << "NAME: " << substring;
                    std::shared_ptr<Menu::MenuItem> item2add = this->findMenuItemByName(substring.toStdString());
                    if(item2add){
                        qDebug() << QString::fromStdString(item2add->getName()) << " selected.";
                        emit addItem2User(item2add);
                    }
                } else {
                    qDebug() << "No tab character found in selected item:" << text;
                }
            }
            this->on_pushButton_return_clicked(); // return to main menu
        } else if (result == QMessageBox::No) {
            qDebug() << "User clicked No button";
        }

    } else {
        QMessageBox::warning(this, "No Item Selected", "No item is selected, before adding to your menu, you should choose items first!");
    }
}


void suggestionScreen::on_pushButton_addAll_clicked()
{
    QList<QListWidgetItem*> selectedItems = ui->listWidget->findItems(QString("*"), Qt::MatchWildcard); // take all items

    if (!selectedItems.isEmpty()) {
        // Extract substring until the first tab character
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
                int total_pass_char = 20;
                int startPos = text.indexOf("-");
                if (startPos == -1) {
                    qDebug() << "Could not find the start of the first '-' sequence";
                }
                int first_index = total_pass_char + startPos;
                int last_index = text.indexOf(QChar(0x200B)); // Find the position of the first tab character
                if (last_index != -1) {
                    QString substring = text.mid(first_index, last_index - first_index); // Extract the substring from the beginning to the tab character
                    // qDebug() << "NAME: " << substring;
                    std::shared_ptr<Menu::MenuItem> item2add = this->findMenuItemByName(substring.toStdString());
                    if (item2add){
                        qDebug() << QString::fromStdString(item2add->getName()) << " selected.";
                        emit addItem2User(item2add);
                    }
                } else {
                    qDebug() << "No tab character found in selected item:" << text;
                }
            }
            this->on_pushButton_return_clicked(); // return to main menu
        } else if (result == QMessageBox::No) {
            qDebug() << "User clicked No button";
        }

    } else {
        QMessageBox::warning(this, "No Item in the List", "No item found in the list, before adding to your menu, you should add some items first!");
    }
}

std::shared_ptr<Menu::MenuItem> suggestionScreen::findMenuItemByName(std::string _name){
    for (auto& item_list : this->available_menu->getMenu()){
        for (auto& item : item_list){
            if (item->getName() == _name){
                return item;
            }
        }
    }
    return nullptr;
}

