/**
  ITU Solar Car Team - Autonomous Weekly Assignment 5
  Author: Yusuf Yıldız
  Date: 12.02.2024
*/

#include "menu.hpp"

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

  void Menu::printMenu() {
    std::cout << "**********Your Menu**********" << std::endl;
    for (int i = 0; i < static_cast<int>(DishType::NUM_DISH_TYPES); i++) {
      DishType type = static_cast<DishType>(i);
      std::string dish_type;
      switch (type) {
        case DishType::STARTER:
          std::cout << "Starters" << std::endl;
          dish_type = "Starters";
          break;
        case DishType::SALAD:
          std::cout << "Salads" << std::endl;
          dish_type = "Salads";
          break;
        case DishType::MAIN_COURSE:
          std::cout << "Main Courses" << std::endl;
          dish_type = "Main Courses";
          break;
        case DishType::DRINK:
          std::cout << "Drinks" << std::endl;
          dish_type = "Drinks";
          break;
        case DishType::APPETIZER:
          std::cout << "Appetizers" << std::endl;
          dish_type = "Appetizers";
          break;
        case DishType::DESSERT:
          std::cout << "Desserts" << std::endl;
          dish_type = "Desserts";
          break;
      }
      if(this->menu[i].empty()){
        std::cout << "----------------" << std::endl;
        std::cout << "Empty Preference for Dish Type: " << dish_type << std::endl;
        std::cout << "----------------" << std::endl;
      } else {
        std::cout << "----------------" << std::endl;
        for (auto &item: this->menu[i]) {
          item->printDish();
          std::cout << "----------------" << std::endl;
        }
      }
    }
    std::cout << "**********End**********" << std::endl;
    std::cout << "**********Overview of Your Menu**********" << std::endl;
    std::cout << "Total Price: " << this->total_price << std::endl;
    std::cout << "----------------" << std::endl;
    this->taste_balance.print();
    std::cout << "**********End of Overview**********" << std::endl;
  }

  void Menu::addMenuItem(std::shared_ptr<MenuItem> _item) {
    if(typeid(*_item) == typeid(Starter)) this->menu[STARTER].push_back(_item);
    else if(typeid(*_item) == typeid(Salads)) this->menu[SALAD].push_back(_item);
    else if(typeid(*_item) == typeid(MainCourse)) this->menu[MAIN_COURSE].push_back(_item);
    else if(typeid(*_item) == typeid(Drinks)) this->menu[DRINK].push_back(_item);
    else if(typeid(*_item) == typeid(Appetizer)) this->menu[APPETIZER].push_back(_item);
    else if(typeid(*_item) == typeid(Desserts)) this->menu[DESSERT].push_back(_item);

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

  std::shared_ptr<MenuItem> Menu::findMenuItemByIndex(int _index) {
    int idx = 1;
    for (int i = 0; i < static_cast<int>(DishType::NUM_DISH_TYPES); i++) {
      if (this->menu[i].empty()) continue;
      else{
        for (auto &item: this->menu[i]) {
          if (idx++ == _index) {
            return item;
          }
        }
      }
    }
    return nullptr;
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