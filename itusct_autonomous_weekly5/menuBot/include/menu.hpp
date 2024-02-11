//
// Created by yildiz on 07.02.2024.
//

#ifndef MENU_BOT_MENU_HPP
#define MENU_BOT_MENU_HPP

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <cmath>

namespace Menu {
enum Taste {
  SWEET,
  SOUR,
  BITTER,
  SALTY,
  SAVORY,
  NUM_TASTES
};

enum DishType {
  STARTER,
  SALAD,
  MAIN_COURSE,
  DRINK,
  APPETIZER,
  DESSERT,
  NUM_DISH_TYPES
};

struct TasteBalance {
  int sweet ;
  int sour;
  int salty;
  int bitter;
  int savory;

  TasteBalance() : sweet(0), sour(0), salty(0), bitter(0), savory(0) {}

  TasteBalance(int _sweet, int _sour, int _salty, int _bitter, int _savory)
          : sweet(_sweet), sour(_sour), salty(_salty), bitter(_bitter), savory(_savory) {};

  TasteBalance(std::vector<int> _taste_balance) : sweet(_taste_balance[SWEET]), sour(_taste_balance[SOUR]),
                                             salty(_taste_balance[SALTY]), bitter(_taste_balance[BITTER]),
                                             savory(_taste_balance[SAVORY]) {};

  TasteBalance(const TasteBalance& _taste_balance) : sweet(_taste_balance.sweet), sour(_taste_balance.sour),
                                             salty(_taste_balance.salty), bitter(_taste_balance.bitter),
                                             savory(_taste_balance.savory) {};

  void print() {
    std::cout << "\tSweet: " << this->sweet << std::endl;
    std::cout << "\tSour: " << this->sour << std::endl;
    std::cout << "\tSalty: " << this->salty << std::endl;
    std::cout << "\tBitter: " << this->bitter << std::endl;
    std::cout << "\tSavory: " << this->savory << std::endl;
  };

  std::vector<int> tasteBalance() {
    return {this->sweet, this->sour, this->salty, this->bitter, this->savory};
  };
};

namespace Extras {
  constexpr double salad_topping = 2.25;
  constexpr double drink_carbonation = 0.5;
  constexpr double drink_alcohol = 2.5;
  constexpr double dessert_chocolate = 1.5;
}; // End namespace Extras

class MenuItem {
private:
  std::string name;
  double price;
  TasteBalance taste_balance;
public:
  MenuItem() : name(""), price(0), taste_balance(TasteBalance()) {};
  MenuItem(std::string _name, double _price, TasteBalance _taste_balance);

  std::string getName() const { return this->name; };
  double getPrice() const { return this->price; };
  TasteBalance getTasteBalance() const { return this->taste_balance; };

  void setName(std::string _name) { this->name = _name; };
  void setPrice(double _price) { this->price = _price; };
  void setTasteBalance(TasteBalance _taste_balance) { this->taste_balance = _taste_balance; };

  virtual void printDish();
};

class Starter : public MenuItem {
private:
  bool is_hot = false;
public:
  Starter(std::string _name, double _price, TasteBalance _taste_balance) : MenuItem(_name, _price, _taste_balance) {};

  bool getIsHot() const { return this->is_hot; };

  void setIsHot(bool _is_hot) { this->is_hot = _is_hot; };

  void printDish() override;
};

class Salads : public MenuItem {
private:
  bool add_topping = false;
  std::string topping = "";
public:
  Salads(std::string _name, double _price, TasteBalance _taste_balance) : MenuItem(_name, _price, _taste_balance) {};

  bool getAddTopping() const { return this->add_topping; };
  std::string getTopping() const { return this->topping; };

  void setAddTopping(bool _add_topping) { this->add_topping = _add_topping; };
  void setTopping(std::string _topping) { this->topping = _topping; };

  void printDish() override;
};

class MainCourse : public MenuItem {
private:
  bool is_vegan = false;
public:
  MainCourse(std::string _name, double _price, TasteBalance _taste_balance) : MenuItem(_name, _price, _taste_balance) {};

  bool getIsVegan() const { return this->is_vegan; };

  void setIsVegan(bool _is_vegan) { this->is_vegan = _is_vegan; };

  void printDish() override;
};

class Drinks : public MenuItem {
private:
  bool is_carbonated = false;
  bool is_alcoholic = false;
public:
  Drinks(std::string _name, double _price, TasteBalance _taste_balance) : MenuItem(_name, _price, _taste_balance) {};


  bool getIsCarbonated() const { return this->is_carbonated; };
  bool getIsAlcoholic() const { return this->is_alcoholic; };

  void setIsCarbonated(bool _is_carbonated) { this->is_carbonated = _is_carbonated; };
  void setIsAlcoholic(bool _is_alcoholic) { this->is_alcoholic = _is_alcoholic; };

  void printDish() override;
};

class Appetizer : public MenuItem {
private:
  bool is_before_main_course = false;
public:
  Appetizer(std::string _name, double _price, TasteBalance _taste_balance) : MenuItem(_name, _price, _taste_balance) {};

  bool getIsBeforeMainCourse() const { return this->is_before_main_course; };

  void setIsBeforeMainCourse(bool _is_before_main_course) { this->is_before_main_course = _is_before_main_course; };

  void printDish() override;
};

class Desserts : public MenuItem {
private:
  bool add_chocolate = false;
public:
  Desserts(std::string _name, double _price, TasteBalance _taste_balance) : MenuItem(_name, _price, _taste_balance) {};

  bool getAddChocolate() const { return this->add_chocolate; };

  void setAddChocolate(bool _add_chocolate) { this->add_chocolate = _add_chocolate; };

  void printDish() override;
};

class Menu{
private:
  std::vector<std::vector<std::shared_ptr<MenuItem>>> menu;
  TasteBalance taste_balance;
  double total_price = 0;
public:
  Menu();
  Menu(std::vector<std::vector<std::shared_ptr<MenuItem>>>& _menu, TasteBalance _taste_balance = TasteBalance(0, 0, 0, 0, 0), double _total_price = 0)
          : menu(_menu), taste_balance(_taste_balance), total_price(_total_price) { this->updateTasteBalanceAndPrice(); };
  std::vector<std::vector<std::shared_ptr<MenuItem>>>& getMenu() { return this->menu; };
  TasteBalance getTasteBalance() const { return this->taste_balance; };
  double getTotalPrice() const { return this->total_price; };

  void setTasteBalance(std::vector<int> _taste_balance) { this->taste_balance = TasteBalance(_taste_balance); };
  void setTotalPrice(double _total_price) { this->total_price = _total_price; };

  void addMenuItem(std::shared_ptr<MenuItem> _item);
  void removeMenuItem(std::string _name);
  void printMenu();
  std::shared_ptr<MenuItem> findMenuItemByIndex(int _index);
  void updateTasteBalanceAndPrice();
};

} // End namespace Menu

#endif //MENU_BOT_MENU_HPP
