/**
  ITU Solar Car Team - Autonomous Weekly Assignment 5
  Author: Yusuf Yıldız
  Date: 12.02.2024
*/

#include "../../include/menu_bot/menu.hpp"
#include <QDebug>

namespace Menu {
  MenuItem::MenuItem(std::string _name, double _price, TasteBalance _taste_balance)
          : name(_name), price(_price), taste_balance(_taste_balance) {};

  void MenuItem::printDish() {
    std::cout << "\tName: " << this->name << std::endl;
    std::cout << "\tPrice: " << this->price << std::endl;
    std::cout << "\tTaste Balance" << std::endl;
    std::cout << "----------------" << std::endl;
    this->taste_balance.print();
  }

  void Starter::printDish() {
    MenuItem::printDish();
    std::cout << "\tHotness Preference: " << (getIsHot() ? "Hot" : "Cold") << std::endl;
  }

  void Salads::printDish() {
    MenuItem::printDish();
    if (getAddTopping()) {
      std::cout << "\tAdditional Topping: " << getTopping() << std::endl;
    } else {
      std::cout << "\tNo Topping Selected" << std::endl;
    }
  }

  void MainCourse::printDish() {
    MenuItem::printDish();
    std::cout << "\tVegan: " << (getIsVegan() ? "Yes" : "No") << std::endl;
  }

  void Drinks::printDish() {
    MenuItem::printDish();
    std::cout << "\tAdditional Carbonation: " << (getIsCarbonated() ? "Yes" : "No") << std::endl;
    std::cout << "\tAdditional Alcohol: " << (getIsAlcoholic() ? "Yes" : "No") << std::endl;
  }

  void Appetizer::printDish() {
    MenuItem::printDish();
    std::cout << "\tService Time Preference: " << (getIsBeforeMainCourse() ? "Before the Main Course" : "After the Main Course") << std::endl;
  }

  void Desserts::printDish() {
    MenuItem::printDish();
    std::cout << "\tAdditional Chocolate: " << (getAddChocolate() ? "Yes" : "No") << std::endl;
  }

  Menu::Menu() {
    this->menu = std::vector<std::vector<std::shared_ptr<MenuItem>>>(static_cast<int>(DishType::NUM_DISH_TYPES));
    setTasteBalance(std::vector<int>{0, 0, 0, 0, 0});
  }

  void Menu::addMenuItem(std::shared_ptr<MenuItem> _item) {
    if(typeid((*_item)) == typeid(Starter)) this->menu[STARTER].push_back(_item);
    else if(typeid((*_item)) == typeid(Salads)) this->menu[SALAD].push_back(_item);
    else if(typeid((*_item)) == typeid(MainCourse)) this->menu[MAIN_COURSE].push_back(_item);
    else if(typeid((*_item)) == typeid(Drinks)) this->menu[DRINK].push_back(_item);
    else if(typeid((*_item)) == typeid(Appetizer)) this->menu[APPETIZER].push_back(_item);
    else if(typeid((*_item)) == typeid(Desserts)) this->menu[DESSERT].push_back(_item);
    this->updateTasteBalanceAndPrice();
  }

  void Menu::removeMenuItem(std::string _name) {
    for (int i = 0; i < static_cast<int>(DishType::NUM_DISH_TYPES); i++) {
      for (auto it = this->menu[i].begin(); it != this->menu[i].end(); it++) {
        if ((*it)->getName() == _name) {
          this->menu[i].erase(it);

          this->updateTasteBalanceAndPrice();
          return;
        }
      }
    }
  }

  void Menu::updateTasteBalanceAndPrice() {
    std::vector<int> taste_bal(static_cast<int>(Taste::NUM_TASTES), 0);
    double tot_price = 0;
    int item_count = 0;
    for (int i = 0; i < static_cast<int>(DishType::NUM_DISH_TYPES); i++) {
      for (auto &item: this->menu[i]) {
        tot_price += item->getPrice();
        item_count++;
        for (int j = 0; j < static_cast<int>(Taste::NUM_TASTES); j++) {
          taste_bal[j] += item->getTasteBalance().tasteBalance()[j];
        }
      }
    }
    if (item_count == 0) {
      this->setTasteBalance(std::vector<int>{0, 0, 0, 0, 0});
      this->setTotalPrice(0);
    } else {
      for (int i = 0; i < static_cast<int>(Taste::NUM_TASTES); i++) {
        taste_bal[i] = std::round(taste_bal[i] /= item_count);
      }
      this->setTasteBalance(taste_bal);
      this->setTotalPrice(tot_price);
    }
  }
} // End namespace Menu
