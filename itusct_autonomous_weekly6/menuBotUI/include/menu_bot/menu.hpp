/**
  ITU Solar Car Team - Autonomous Weekly Assignment 5
  Author: Yusuf Yıldız
  Date: 12.02.2024
*/

#ifndef MENU_BOT_MENU_HPP
#define MENU_BOT_MENU_HPP

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <cmath>

/**
 * @brief The Menu namespace contains the classes and enums that are used to represent the menu of a restaurant and the user.
 */
namespace Menu {
  /**
   * @brief The Taste enum is used to represent the different tastes that a dish can have.
   */
  enum Taste {
    SWEET,
    SOUR,
    BITTER,
    SALTY,
    SAVORY,
    NUM_TASTES
  };

  /**
   * @brief The DishType enum is used to represent the different types of dishes that can be in the menu.
   */
  enum DishType {
    STARTER,
    SALAD,
    MAIN_COURSE,
    DRINK,
    APPETIZER,
    DESSERT,
    NUM_DISH_TYPES
  };

  /**
   * @brief The TasteBalance struct is used to represent the taste balance of a dish.
   */
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
    // Explicitly defined copy assignment operator
    TasteBalance& operator=(const TasteBalance& other) {
        if (this != &other) {
            sweet = other.sweet;
            sour = other.sour;
            salty = other.salty;
            bitter = other.bitter;
            savory = other.savory;
        }
        return *this;
    }

    /**
     * @brief Prints the taste balance of the dish.
     */
    void print() {
      std::cout << "\tSweet: " << this->sweet << std::endl;
      std::cout << "\tSour: " << this->sour << std::endl;
      std::cout << "\tSalty: " << this->salty << std::endl;
      std::cout << "\tBitter: " << this->bitter << std::endl;
      std::cout << "\tSavory: " << this->savory << std::endl;
    };

    /**
     * @brief Returns the taste balance as a vector of integers.
     * @return The taste balance as a vector of integers.
     */
    std::vector<int> tasteBalance() {
      return {this->sweet, this->sour, this->salty, this->bitter, this->savory};
    };
  };

  /**
   * @brief The Extras namespace contains the extra costs for different types extra services.
   */
  namespace Extras {
    constexpr double salad_topping = 2.25;
    constexpr double drink_carbonation = 0.5;
    constexpr double drink_alcohol = 2.5;
    constexpr double dessert_chocolate = 1.5;
  }; // End namespace Extras

  /**
   * @brief The MenuItem class is a base class for all types of dishes.
   */
  class MenuItem {
  private:
    std::string name;
    double price;
    TasteBalance taste_balance;
  public:
    MenuItem() : name(""), price(0), taste_balance(TasteBalance()) {};
    MenuItem(std::string _name, double _price, TasteBalance _taste_balance);

    // Getter functions for private members
    std::string getName() const { return this->name; };
    double getPrice() const { return this->price; };
    TasteBalance getTasteBalance() const { return this->taste_balance; };

    // Setter functions for private members
    void setName(std::string _name) { this->name = _name; };
    void setPrice(double _price) { this->price = _price; };
    void setTasteBalance(TasteBalance _taste_balance) { this->taste_balance = _taste_balance; };

    /**
     * @brief This is a virtual function to print the dishes, and must be override.
     */
    virtual void printDish();
  };

  /**
   * @brief The Starter class is a derived class from MenuItem, and represents the starter dishes in the menu.
   */
  class Starter : public MenuItem {
  private:
    bool is_hot = false;
  public:
    Starter(std::string _name, double _price, TasteBalance _taste_balance) : MenuItem(_name, _price, _taste_balance) {};

    // Getter functions for private members
    bool getIsHot() const { return this->is_hot; };

    // Setter functions for private members
    void setIsHot(bool _is_hot) { this->is_hot = _is_hot; };

    void printDish() override;
  };

  /**
   * @brief The Salads class is a derived class from MenuItem, and represents the salad dishes in the menu.
   */
  class Salads : public MenuItem {
  private:
    bool add_topping = false;
    std::string topping = "";
  public:
    Salads(std::string _name, double _price, TasteBalance _taste_balance) : MenuItem(_name, _price, _taste_balance) {};

    // Getter functions for private members
    bool getAddTopping() const { return this->add_topping; };
    std::string getTopping() const { return this->topping; };

    // Setter functions for private members
    void setAddTopping(bool _add_topping) { this->add_topping = _add_topping; };
    void setTopping(std::string _topping) { this->topping = _topping; };

    void printDish() override;
  };

  /**
   * @brief The MainCourse class is a derived class from MenuItem, and represents the main course dishes in the menu.
   */
  class MainCourse : public MenuItem {
  private:
    bool is_vegan = false;
  public:
    MainCourse(std::string _name, double _price, TasteBalance _taste_balance) : MenuItem(_name, _price, _taste_balance) {};

    // Getter functions for private members
    bool getIsVegan() const { return this->is_vegan; };

    // Setter functions for private members
    void setIsVegan(bool _is_vegan) { this->is_vegan = _is_vegan; };

    void printDish() override;
  };

  /**
   * @brief The Drinks class is a derived class from MenuItem, and represents the drink dishes in the menu.
   */
  class Drinks : public MenuItem {
  private:
    bool is_carbonated = false;
    bool is_alcoholic = false;
  public:
    Drinks(std::string _name, double _price, TasteBalance _taste_balance) : MenuItem(_name, _price, _taste_balance) {};

    // Getter functions for private members
    bool getIsCarbonated() const { return this->is_carbonated; };
    bool getIsAlcoholic() const { return this->is_alcoholic; };

    // Setter functions for private members
    void setIsCarbonated(bool _is_carbonated) { this->is_carbonated = _is_carbonated; };
    void setIsAlcoholic(bool _is_alcoholic) { this->is_alcoholic = _is_alcoholic; };

    void printDish() override;
  };

  /**
   * @brief The Appetizer class is a derived class from MenuItem, and represents the appetizer dishes in the menu.
   */
  class Appetizer : public MenuItem {
  private:
    bool is_before_main_course = false;
  public:
    Appetizer(std::string _name, double _price, TasteBalance _taste_balance) : MenuItem(_name, _price, _taste_balance) {};

    // Getter functions for private members
    bool getIsBeforeMainCourse() const { return this->is_before_main_course; };

    // Setter functions for private members
    void setIsBeforeMainCourse(bool _is_before_main_course) { this->is_before_main_course = _is_before_main_course; };

    void printDish() override;
  };

  /**
   * @brief The Desserts class is a derived class from MenuItem, and represents the dessert dishes in the menu.
   */
  class Desserts : public MenuItem {
  private:
    bool add_chocolate = false;
  public:
    Desserts(std::string _name, double _price, TasteBalance _taste_balance) : MenuItem(_name, _price, _taste_balance) {};

    // Getter functions for private members
    bool getAddChocolate() const { return this->add_chocolate; };

    // Setter functions for private members
    void setAddChocolate(bool _add_chocolate) { this->add_chocolate = _add_chocolate; };

    void printDish() override;
  };

  /**
   * @brief The Menu class is used to represent the menu of a restaurant.
   */
  class Menu{
  private:
    std::vector<std::vector<std::shared_ptr<MenuItem>>> menu;
    TasteBalance taste_balance;
    double total_price = 0;
  public:
    Menu();
    Menu(std::vector<std::vector<std::shared_ptr<MenuItem>>>& _menu, TasteBalance _taste_balance = TasteBalance(0, 0, 0, 0, 0), double _total_price = 0)
            : menu(_menu), taste_balance(_taste_balance), total_price(_total_price) { this->updateTasteBalanceAndPrice(); };

    // Getter functions for private members
    std::vector<std::vector<std::shared_ptr<MenuItem>>>& getMenu() { return this->menu; };
    TasteBalance getTasteBalance() const { return this->taste_balance; };
    double getTotalPrice() const { return this->total_price; };

    // Setter functions for private members
    void setTasteBalance(std::vector<int> _taste_balance) { this->taste_balance = TasteBalance(_taste_balance); };
    void setTotalPrice(double _total_price) { this->total_price = _total_price; };

    // /**
    //  * @brief Prints the menu and the overview of the menu.
    //  */
    // void printMenu();

    /**
     * @brief Adds a menu item to the menu.
     * @param _item The menu item to add to the menu.
     */
    void addMenuItem(std::shared_ptr<MenuItem> _item);

    /**
     * @brief Removes a menu item from the menu.
     * @param _name The name of the menu item to remove from the menu.
     */
    void removeMenuItem(std::string _name);

    // /**
    //  * @brief Finds a menu item by its index in the menu.
    //  * @param _index The index of the menu item to find.
    //  * @return A shared pointer to the menu item if found, nullptr otherwise.
    //  */
    // std::shared_ptr<MenuItem> findMenuItemByIndex(int _index);

    /**
     * @brief Updates the taste balance and total price of the menu after each transaction.
     */
    void updateTasteBalanceAndPrice();
  };
} // End namespace Menu

#endif //MENU_BOT_MENU_HPP
