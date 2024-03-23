#include "../../include/headers/dish_widget.hpp"

DishWidget::DishWidget(std::shared_ptr<Menu::MenuItem> menuItem, Menu::DishType dishType, QVector<QVector<QCheckBox*>> &checkboxMatrix, QWidget *parent) : QWidget(parent)
{
    this->dishLabel = new QLabel(QString::fromStdString(menuItem->getName()));

    // Initialize checkboxes based on dish type
    switch (dishType) {
    case Menu::DishType::STARTER:
        this->checkbox1 = new QCheckBox("Serve the Starter Hot");
        this->checkbox2 = nullptr; // No salad checkbox for starters
        this->checkbox1->setObjectName("checkbox_starter_hot");

        if (std::shared_ptr<Menu::Starter> starterPtr= std::dynamic_pointer_cast<Menu::Starter>(menuItem)){
            if (starterPtr->getIsHot()) this->checkbox1->setChecked(true);
        }
        checkboxMatrix[Menu::DishType::STARTER].push_back(this->checkbox1);
        break;
    case Menu::DishType::SALAD:
        this->checkbox1 = new QCheckBox("Additional Topping to Salad($2.25)"); // No starter checkbox for salads
        this->checkbox2 = nullptr;
        this->checkbox1->setObjectName("checkbox_salad_topping");

        if (std::shared_ptr<Menu::Salads> saladPtr= std::dynamic_pointer_cast<Menu::Salads>(menuItem)){
            if (saladPtr->getAddTopping()) this->checkbox1->setChecked(true);
        }
        checkboxMatrix[Menu::DishType::SALAD].push_back(this->checkbox1);
        break;
    case Menu::DishType::MAIN_COURSE:
        this->checkbox1 = new QCheckBox("Vegan Option for Main Course"); // No starter checkbox for salads
        this->checkbox2 = nullptr;
        this->checkbox1->setObjectName("checkbox_main_vegan");

        if (std::shared_ptr<Menu::MainCourse> mainPtr= std::dynamic_pointer_cast<Menu::MainCourse>(menuItem)){
            if (mainPtr->getIsVegan()) this->checkbox1->setChecked(true);
        }
        checkboxMatrix[Menu::DishType::MAIN_COURSE].push_back(this->checkbox1);
        break;
    case Menu::DishType::DRINK:
        this->checkbox1 = new QCheckBox("Additional Carbonation to Your Drink($0.5)"); // No starter checkbox for salads
        this->checkbox2 = new QCheckBox("Additional Alcohol to Your Drink($2.5)");
        this->checkbox1->setObjectName("checkbox_drink_carbonate");
        this->checkbox2->setObjectName("checkbox_main_alcohol");

        if (std::shared_ptr<Menu::Drinks> drinkPtr= std::dynamic_pointer_cast<Menu::Drinks>(menuItem)){
            if (drinkPtr->getIsCarbonated()) this->checkbox1->setChecked(true);
        }
        checkboxMatrix[Menu::DishType::DRINK].push_back(this->checkbox1);

        if (std::shared_ptr<Menu::Drinks> drinkPtr= std::dynamic_pointer_cast<Menu::Drinks>(menuItem)){
            if (drinkPtr->getIsAlcoholic()) this->checkbox2->setChecked(true);
        }
        checkboxMatrix[Menu::DishType::NUM_DISH_TYPES].push_back(this->checkbox2); // this checkboxes will be kept at the last index
        break;
    case Menu::DishType::APPETIZER:
        this->checkbox1 = new QCheckBox("Serve Your Appetizer Before the Main Course"); // No starter checkbox for salads
        this->checkbox2 = nullptr;
        this->checkbox1->setObjectName("checkbox_appetizer_before");

        if (std::shared_ptr<Menu::Appetizer> appetizerPtr= std::dynamic_pointer_cast<Menu::Appetizer>(menuItem)){
            if (appetizerPtr->getIsBeforeMainCourse()) this->checkbox1->setChecked(true);
        }
        checkboxMatrix[Menu::DishType::APPETIZER].push_back(this->checkbox1);
        break;
    case Menu::DishType::DESSERT:
        this->checkbox1 = new QCheckBox("Additional Chocolate to Your Dessert($1.5)"); // No starter checkbox for salads
        this->checkbox2 = nullptr;
        this->checkbox1->setObjectName("checkbox_dessert_chocolate");

        if (std::shared_ptr<Menu::Desserts> dessertPtr= std::dynamic_pointer_cast<Menu::Desserts>(menuItem)){
            if (dessertPtr->getAddChocolate()) this->checkbox1->setChecked(true);
        }
        checkboxMatrix[Menu::DishType::DESSERT].push_back(this->checkbox1);
        break;
    }

    layout = new QGridLayout;
    layout->addWidget(this->dishLabel, 0, 0, Qt::AlignLeft);
    if (this->checkbox1) {
        this->checkbox1->setStyleSheet("QCheckBox { border-top: 1px solid black; border-bottom: 1px solid black; }");
        layout->addWidget(this->checkbox1, 0, 1);
    }
    if (this->checkbox2) {
        this->checkbox2->setStyleSheet("QCheckBox { border-top: 1px solid black; border-bottom: 1px solid black; }");
        layout->addWidget(this->checkbox2, 1, 1);
    }

    // Add left and right borders to the label
    this->dishLabel->setStyleSheet("QLabel { border-bottom: 1px solid black; }");

    setLayout(layout);
}
