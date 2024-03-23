#include "../../include/headers/approve_menu.hpp"
#include "ui_approve_menu.h"
#include "../../include/headers/dish_widget.hpp"

approveMenu::approveMenu(QWidget *parent, User::User *_user, QString _window_name)
    : QDialog(parent)
    , ui(new Ui::approveMenu)
{
    this->user = _user;
    ui->setupUi(this);
    setWindowTitle(_window_name);
    QPixmap pix(":icons/gae_logo.png");
    this->ui->label_icon->setPixmap(pix.scaled(150, 150, Qt::KeepAspectRatio));
    this->ui->label_icon->setAlignment(Qt::AlignCenter);

    this->checkboxMatrix.resize(static_cast<int>(Menu::DishType::NUM_DISH_TYPES + 1)); // +1 for additional checkbox in the drink
}

void approveMenu::addDishWidgets(){
    // Clear the existing layout
    QLayoutItem* item;
    while ((item = ui->verticalLayout_items->takeAt(0)) != nullptr) {
        delete item->widget(); // Delete the widget
        delete item; // Delete the layout item
    }

    for (int i = 0; i < this->checkboxMatrix.size(); i++){
        this->checkboxMatrix[i].clear();
    }

    for (int i = 0; i < static_cast<int>(Menu::DishType::NUM_DISH_TYPES); i++) {
        Menu::DishType type = static_cast<Menu::DishType>(i);
        // std::string dish_type;
        switch (type) {
        case Menu::DishType::STARTER:

            if(this->user->getMenu()->getMenu()[i].empty()){
                // qDebug() << "Starter Empty";
            } else {
                for (auto &item: this->user->getMenu()->getMenu()[i]) {
                    DishWidget *dishWidget = new DishWidget(item, Menu::DishType::STARTER, this->checkboxMatrix);
                    // qDebug() << "Starter is not empty, adding";
                    ui->verticalLayout_items->addWidget(dishWidget);
                }
            }
            break;
        case Menu::DishType::SALAD:

            if(this->user->getMenu()->getMenu()[i].empty()){
                // qDebug() << "Salad Empty";
            } else {
                for (auto &item: this->user->getMenu()->getMenu()[i]) {
                    DishWidget *dishWidget = new DishWidget(item, Menu::DishType::SALAD, this->checkboxMatrix);
                    // qDebug() << "Salad is not empty, adding";
                    ui->verticalLayout_items->addWidget(dishWidget);
                }
            }
            break;

        case Menu::DishType::MAIN_COURSE:

            if(this->user->getMenu()->getMenu()[i].empty()){
                // qDebug() << "Main Course Empty";
            } else {
                for (auto &item: this->user->getMenu()->getMenu()[i]) {
                    DishWidget *dishWidget = new DishWidget(item, Menu::DishType::MAIN_COURSE, this->checkboxMatrix);
                    // qDebug() << "Main Course is not empty, adding";
                    ui->verticalLayout_items->addWidget(dishWidget);
                }
            }
            break;

        case Menu::DishType::DRINK:

            if(this->user->getMenu()->getMenu()[i].empty()){
                // qDebug() << "Drink Empty";
            } else {
                for (auto &item: this->user->getMenu()->getMenu()[i]) {
                    DishWidget *dishWidget = new DishWidget(item, Menu::DishType::DRINK, this->checkboxMatrix);
                    // qDebug() << "Drink is not empty, adding";
                    ui->verticalLayout_items->addWidget(dishWidget);
                }
            }
            break;

        case Menu::DishType::APPETIZER:

            if(this->user->getMenu()->getMenu()[i].empty()){
                // qDebug() << "Appetizer Empty";
            } else {
                for (auto &item: this->user->getMenu()->getMenu()[i]) {
                    DishWidget *dishWidget = new DishWidget(item, Menu::DishType::APPETIZER, this->checkboxMatrix);
                    // qDebug() << "Appetizer is not empty, adding";
                    ui->verticalLayout_items->addWidget(dishWidget);
                }
            }
            break;

        case Menu::DishType::DESSERT:

            if(this->user->getMenu()->getMenu()[i].empty()){
                // qDebug() << "Dessert Empty";
            } else {
                for (auto &item: this->user->getMenu()->getMenu()[i]) {
                    DishWidget *dishWidget = new DishWidget(item, Menu::DishType::DESSERT, this->checkboxMatrix);
                    qDebug() << "Dessert is not empty, adding";
                    ui->verticalLayout_items->addWidget(dishWidget);
                }
            }
            break;
        }

    }

    setLayout(ui->verticalLayout_items);
}

approveMenu::~approveMenu()
{
    delete ui;
}

void approveMenu::on_pushButton_approve_clicked()
{
    // qDebug() << "APPROVING";
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("Menu Approval Validation");
    msgBox.setText("Are you sure to approve your preferences?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::No);

    int result = msgBox.exec();
    if (result == QMessageBox::Yes) {
        for (int i = 0; i < static_cast<int>(Menu::DishType::NUM_DISH_TYPES) + 1; i++) {
            Menu::DishType type = static_cast<Menu::DishType>(i);
            switch(type) {
            case(Menu::DishType::STARTER):
                for (int i = 0; i < this->checkboxMatrix[Menu::DishType::STARTER].size(); ++i) {
                    if (this->checkboxMatrix[Menu::DishType::STARTER][i]->isChecked()) {
                        if (std::shared_ptr<Menu::Starter> starterPtr= std::dynamic_pointer_cast<Menu::Starter>(this->user->getMenu()->getMenu()[Menu::DishType::STARTER][i])){
                            if (!starterPtr->getIsHot()) starterPtr->setIsHot(true);
                        }
                    } else {
                        if (std::shared_ptr<Menu::Starter> starterPtr= std::dynamic_pointer_cast<Menu::Starter>(this->user->getMenu()->getMenu()[Menu::DishType::STARTER][i])){
                            if (starterPtr->getIsHot()) starterPtr->setIsHot(false);
                        }
                    }
                }
                break;
            case(Menu::DishType::SALAD):
                for (int i = 0; i < this->checkboxMatrix[Menu::DishType::SALAD].size(); ++i) {
                    if (this->checkboxMatrix[Menu::DishType::SALAD][i]->isChecked()) {
                        if (std::shared_ptr<Menu::Salads> saladPtr= std::dynamic_pointer_cast<Menu::Salads>(this->user->getMenu()->getMenu()[Menu::DishType::SALAD][i])){
                            if(!saladPtr->getAddTopping()){
                                saladPtr->setAddTopping(true);
                                saladPtr->setTopping("Topping selected.");
                                this->user->getMenu()->setTotalPrice(this->user->getMenu()->getTotalPrice() + Menu::Extras::salad_topping);
                            }
                        }
                    } else {
                        if (std::shared_ptr<Menu::Salads> saladPtr= std::dynamic_pointer_cast<Menu::Salads>(this->user->getMenu()->getMenu()[Menu::DishType::SALAD][i])){
                            if(saladPtr->getAddTopping()){
                                saladPtr->setAddTopping(false);
                                saladPtr->setTopping("No Topping Selected");
                                this->user->getMenu()->setTotalPrice(this->user->getMenu()->getTotalPrice() - Menu::Extras::salad_topping);
                            }
                        }
                    }
                }
                break;
            case(Menu::DishType::MAIN_COURSE):
                for (int i = 0; i < this->checkboxMatrix[Menu::DishType::MAIN_COURSE].size(); ++i) {
                    if (this->checkboxMatrix[Menu::DishType::MAIN_COURSE][i]->isChecked()) {
                        if (std::shared_ptr<Menu::MainCourse> mainPtr= std::dynamic_pointer_cast<Menu::MainCourse>(this->user->getMenu()->getMenu()[Menu::DishType::MAIN_COURSE][i])){
                            if (!mainPtr->getIsVegan()) mainPtr->setIsVegan(true);
                        }
                    } else {
                        if (std::shared_ptr<Menu::MainCourse> mainPtr= std::dynamic_pointer_cast<Menu::MainCourse>(this->user->getMenu()->getMenu()[Menu::DishType::MAIN_COURSE][i])){
                            if (mainPtr->getIsVegan()) mainPtr->setIsVegan(false);
                        }
                    }
                }
                break;
            case(Menu::DishType::DRINK):
                for (int i = 0; i < this->checkboxMatrix[Menu::DishType::DRINK].size(); ++i) {
                    if (this->checkboxMatrix[Menu::DishType::DRINK][i]->isChecked()) {
                        if (std::shared_ptr<Menu::Drinks> drinkPtr= std::dynamic_pointer_cast<Menu::Drinks>(this->user->getMenu()->getMenu()[Menu::DishType::DRINK][i])){
                            if (!drinkPtr->getIsCarbonated()){
                                drinkPtr->setIsCarbonated(true);
                                this->user->getMenu()->setTotalPrice(this->user->getMenu()->getTotalPrice() + Menu::Extras::drink_carbonation);
                            }
                        }
                    } else {
                        if (std::shared_ptr<Menu::Drinks> drinkPtr= std::dynamic_pointer_cast<Menu::Drinks>(this->user->getMenu()->getMenu()[Menu::DishType::DRINK][i])){
                            if (drinkPtr->getIsCarbonated()){
                                drinkPtr->setIsCarbonated(false);
                                this->user->getMenu()->setTotalPrice(this->user->getMenu()->getTotalPrice() - Menu::Extras::drink_carbonation);
                            }
                        }
                    }
                }
                break;
            case(Menu::DishType::APPETIZER):
                for (int i = 0; i < this->checkboxMatrix[Menu::DishType::APPETIZER].size(); ++i) {
                    if (this->checkboxMatrix[Menu::DishType::APPETIZER][i]->isChecked()) {
                        if (std::shared_ptr<Menu::Appetizer> appetizerPtr= std::dynamic_pointer_cast<Menu::Appetizer>(this->user->getMenu()->getMenu()[Menu::DishType::APPETIZER][i])){
                            if (!appetizerPtr->getIsBeforeMainCourse()) appetizerPtr->setIsBeforeMainCourse(true);
                        }
                    } else {
                        if (std::shared_ptr<Menu::Appetizer> appetizerPtr= std::dynamic_pointer_cast<Menu::Appetizer>(this->user->getMenu()->getMenu()[Menu::DishType::APPETIZER][i])){
                            if (appetizerPtr->getIsBeforeMainCourse()) appetizerPtr->setIsBeforeMainCourse(false);
                        }
                    }
                }
                break;
            case(Menu::DishType::DESSERT):
                for (int i = 0; i < this->checkboxMatrix[Menu::DishType::DESSERT].size(); ++i) {
                    if (this->checkboxMatrix[Menu::DishType::DESSERT][i]->isChecked()) {
                        if (std::shared_ptr<Menu::Desserts> dessertPtr= std::dynamic_pointer_cast<Menu::Desserts>(this->user->getMenu()->getMenu()[Menu::DishType::DESSERT][i])){
                            if (!dessertPtr->getAddChocolate()){
                                dessertPtr->setAddChocolate(true);
                                this->user->getMenu()->setTotalPrice(this->user->getMenu()->getTotalPrice() + Menu::Extras::dessert_chocolate);
                            }
                        }
                    } else {
                        if (std::shared_ptr<Menu::Desserts> dessertPtr= std::dynamic_pointer_cast<Menu::Desserts>(this->user->getMenu()->getMenu()[Menu::DishType::DESSERT][i])){
                            if (dessertPtr->getAddChocolate()){
                                dessertPtr->setAddChocolate(false);
                                this->user->getMenu()->setTotalPrice(this->user->getMenu()->getTotalPrice() - Menu::Extras::dessert_chocolate);
                            }
                        }
                    }
                }
                break;
            case(Menu::DishType::NUM_DISH_TYPES):
                for (int i = 0; i < this->checkboxMatrix[Menu::DishType::NUM_DISH_TYPES].size(); ++i) {
                    if (this->checkboxMatrix[Menu::DishType::NUM_DISH_TYPES][i]->isChecked()) {
                        if (std::shared_ptr<Menu::Drinks> drinkPtr= std::dynamic_pointer_cast<Menu::Drinks>(this->user->getMenu()->getMenu()[Menu::DishType::DRINK][i])){
                            if (!drinkPtr->getIsAlcoholic()){
                                drinkPtr->setIsAlcoholic(true);
                                this->user->getMenu()->setTotalPrice(this->user->getMenu()->getTotalPrice() + Menu::Extras::drink_alcohol);
                            }
                        }
                    } else {
                        if (std::shared_ptr<Menu::Drinks> drinkPtr= std::dynamic_pointer_cast<Menu::Drinks>(this->user->getMenu()->getMenu()[Menu::DishType::DRINK][i])){
                            if (drinkPtr->getIsAlcoholic()){
                                drinkPtr->setIsAlcoholic(false);
                                this->user->getMenu()->setTotalPrice(this->user->getMenu()->getTotalPrice() - Menu::Extras::drink_alcohol);
                            }
                        }
                    }
                }
                break;
            }
        }
    } else if (result == QMessageBox::No) {
        qDebug() << "User clicked No button";
    }

    this->on_pushButton_return_clicked();
}

void approveMenu::on_pushButton_return_clicked()
{
    qDebug() << "Return to User Menu from approveMenu";
    this->hide();
    emit return_userMenu();
}

