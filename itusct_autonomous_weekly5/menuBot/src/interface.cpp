/**
  ITU Solar Car Team - Autonomous Weekly Assignment 5
  Author: Yusuf Yıldız
  Date: 12.02.2024
*/

#include "interface.hpp"

namespace Interface{
  void readFromJSON(std::string _file_name, std::vector<std::vector<std::shared_ptr<Menu::MenuItem>>>& _menu) {
    std::ifstream file(_file_name, std::ios::in);
    nlohmann::json menu_data;
    if (file.is_open()) {
      menu_data = nlohmann::json::parse(file);
    } else {
      std::cerr << "Error: Failed to open file " << _file_name << std::endl;
    }

    for (const std::string &category: {"starters", "salads", "main_courses", "drinks", "appetizers", "desserts"}) {
      for (const auto &item: menu_data[category]) {
        nlohmann::json taste_balance = item["taste_balance"];
        int sweet = taste_balance["sweet"];
        int sour = taste_balance["sour"];
        int bitter = taste_balance["bitter"];
        int salty = taste_balance["salty"];
        int savory = taste_balance["savory"];
        Menu::TasteBalance tasteBalance(sweet, sour, bitter, salty, savory);
        if (category == "starters") {
          std::shared_ptr<Menu::MenuItem> menuItem = std::make_shared<Menu::Starter>(item["name"], item["price"], tasteBalance);
          _menu[Menu::STARTER].push_back(menuItem);
        } else if (category == "salads") {
          std::shared_ptr<Menu::MenuItem> menuItem = std::make_shared<Menu::Salads>(item["name"], item["price"], tasteBalance);
          _menu[Menu::SALAD].push_back(menuItem);
        } else if (category == "main_courses") {
          std::shared_ptr<Menu::MenuItem> menuItem = std::make_shared<Menu::MainCourse>(item["name"], item["price"], tasteBalance);
          _menu[Menu::MAIN_COURSE].push_back(menuItem);
        } else if (category == "drinks") {
          std::shared_ptr<Menu::MenuItem> menuItem = std::make_shared<Menu::Drinks>(item["name"], item["price"], tasteBalance);
          _menu[Menu::DRINK].push_back(menuItem);
        } else if (category == "appetizers") {
          std::shared_ptr<Menu::MenuItem> menuItem = std::make_shared<Menu::Appetizer>(item["name"], item["price"], tasteBalance);
          _menu[Menu::APPETIZER].push_back(menuItem);
        } else if (category == "desserts") {
          std::shared_ptr<Menu::MenuItem> menuItem = std::make_shared<Menu::Desserts>(item["name"], item["price"], tasteBalance);
          _menu[Menu::DESSERT].push_back(menuItem);
        }
      }
    }
  }

  bool addItemByType(User::User& _user, std::shared_ptr<Menu::Menu>& _available_menu, Menu::DishType _type) {
    int index = 1;
    std::cout << "++++++++++++++++++++" << std::endl;

    for (auto& item: _available_menu->getMenu()[static_cast<int>(_type)]) {
      std::cout << index++ << ". ";
      item->printDish();
      std::cout << "++++++++++++++++++++" << std::endl;
    }

    auto is_numeric = [](const std::string &str) -> bool { // check whether the string is numeric or not, this function also controls the negative numbers
      return !str.empty() && std::find_if(str.begin(),
                                          str.end(), [](unsigned char c) { return !std::isdigit(c); }) == str.end(); // control the string digit by digit to be numeric
    };

    int choice;
    std::cout << "Which item would you like to add?" << std::endl;
    std::string choice_str;
    std::cin >> choice_str;
    if (is_numeric(choice_str)) {
      if(std::stoi(choice_str) < 1 || std::stoi(choice_str) > _available_menu->getMenu()[static_cast<int>(_type)].size()){
        std::cout << "Invalid input. Please enter a valid number." << std::endl;
        sleep(1);
        return false;
      }
      choice = std::stoi(choice_str);
    } else {
      std::cout << "Invalid input. Please enter a number." << std::endl;
      sleep(1);
      return false;
    }

    std::shared_ptr<Menu::MenuItem> newItem = _available_menu->getMenu()[static_cast<int>(_type)][choice - 1];
    std::cout << "Chosen item is: " << std::endl;
    std::cout << "++++++++++++++++++++" << std::endl;
    newItem->printDish();
    std::cout << "++++++++++++++++++++" << std::endl;
    std::cout << "\nAre you sure to add this item?(Y or N)" << std::endl;
    char confirm;
    std::cin >> confirm;
    if (confirm == 'Y' || confirm == 'y') {
      _user.getMenu()->addMenuItem(newItem);
      return true;
    } else {
      return false;
    }
  }

  double calculateDistance(const Menu::TasteBalance& _balance1, const Menu::TasteBalance& _balance2) {
    return std::sqrt(std::pow(_balance1.sweet - _balance2.sweet, 2) +
                     std::pow(_balance1.sour - _balance2.sour, 2) +
                     std::pow(_balance1.salty - _balance2.salty, 2) +
                     std::pow(_balance1.bitter - _balance2.bitter, 2) +
                     std::pow(_balance1.savory - _balance2.savory, 2));
  }

  double calculateCovariance(const Menu::TasteBalance& _balance1, const Menu::TasteBalance& _balance2) {
    double mean1 = (_balance1.sweet + _balance1.sour + _balance1.salty + _balance1.bitter + _balance1.savory) / 5.0;
    double mean2 = (_balance2.sweet + _balance2.sour + _balance2.salty + _balance2.bitter + _balance2.savory) / 5.0;

    double covariance = ((_balance1.sweet - mean1) * (_balance2.sweet - mean2) +
                         (_balance1.sour - mean1) * (_balance2.sour - mean2) +
                         (_balance1.salty - mean1) * (_balance2.salty - mean2) +
                         (_balance1.bitter - mean1) * (_balance2.bitter - mean2) +
                         (_balance1.savory - mean1) * (_balance2.savory - mean2)) / 5.0;

    return covariance;
  }

  void generatePermutations(std::vector<std::vector<int>>& _permutations, std::vector<int>& _indices, int _level, const std::vector<int>& _max_sizes){
    if (_level == _indices.size()) {
      _permutations.push_back(_indices);
      return;
    }
    for (int i = 0; i <= _max_sizes[_level]; ++i) {
      _indices[_level] = i;
      generatePermutations(_permutations, _indices, _level + 1, _max_sizes);
    }
  }

  std::vector<std::shared_ptr<Menu::Menu>> generateMenuItemPermutations(std::shared_ptr<Menu::Menu>& _menu){

    std::vector<std::shared_ptr<Menu::Menu>> all_menu_item_permutations;
    std::vector<std::vector<int>> all_permutations;
    std::vector<int> indices(_menu->getMenu().size(), 0);
    std::vector<int> max_sizes(_menu->getMenu().size());
    for (int i = 0; i < static_cast<int>(Menu::DishType::NUM_DISH_TYPES); i++)  {
      max_sizes[i] = _menu->getMenu()[i].size() - 1;
    }
    generatePermutations(all_permutations, indices, 0, max_sizes);

    for (int i = 0; i < all_permutations.size(); i++) {
      std::vector<std::vector<std::shared_ptr<Menu::MenuItem>>> permutation_menu(static_cast<int>(Menu::DishType::NUM_DISH_TYPES));
      for (int j = 0; j < static_cast<int>(Menu::DishType::NUM_DISH_TYPES); j++) {
        permutation_menu[j].push_back(_menu->getMenu()[j][all_permutations[i][j]]);
      }
      all_menu_item_permutations.push_back(std::make_shared<Menu::Menu>(permutation_menu));
    }

    return all_menu_item_permutations;
  }

  std::shared_ptr<Menu::MenuItem> suggestMenuItem(std::shared_ptr<Menu::Menu>& _available_menu, Menu::TasteBalance _taste_balance, Menu::DishType _type) {
    double minDistance = std::numeric_limits<double>::max();
    std::shared_ptr<Menu::MenuItem> suggested_menu_item;

    for (auto& item : _available_menu->getMenu()[static_cast<int>(_type)]){
      double distance = calculateDistance(_taste_balance, item->getTasteBalance());
      if (distance < minDistance) { // if the distance is less than the minimum distance, then update the minimum distance and the suggested menu item
        minDistance = distance;
        suggested_menu_item = item;
      }
      else if (distance == minDistance) { // if the distance is equal, then compare the covariances
        double currentCovariance = calculateCovariance(_taste_balance, item->getTasteBalance());
        double suggestedCovariance = calculateCovariance(_taste_balance, suggested_menu_item->getTasteBalance());
        if (currentCovariance < suggestedCovariance) {
          suggested_menu_item = item;
        } else {
          continue;
        }
      }
    }

    return suggested_menu_item;
  }

  std::shared_ptr<Menu::Menu> suggestFullMenu(std::shared_ptr<Menu::Menu>& _available_menu, Menu::TasteBalance _taste_balance) {
    std::vector<std::shared_ptr<Menu::Menu>> all_menu_item_permutations = generateMenuItemPermutations(_available_menu);
    std::shared_ptr<Menu::Menu> suggested_menu;
    double minDistance = std::numeric_limits<double>::max();

    for (auto& permutation: all_menu_item_permutations) {
      Menu::TasteBalance permutationBalance;
      for (auto& menu: permutation->getMenu()) {
        for(auto& menu_item: menu) {
          permutationBalance.sweet += menu_item->getTasteBalance().sweet;
          permutationBalance.sour += menu_item->getTasteBalance().sour;
          permutationBalance.salty += menu_item->getTasteBalance().salty;
          permutationBalance.bitter += menu_item->getTasteBalance().bitter;
          permutationBalance.savory += menu_item->getTasteBalance().savory;
        }
      }
      permutationBalance.sweet /= permutation->getMenu().size();
      permutationBalance.sour /= permutation->getMenu().size();
      permutationBalance.salty /= permutation->getMenu().size();
      permutationBalance.bitter /= permutation->getMenu().size();
      permutationBalance.savory /= permutation->getMenu().size();

      double distance = calculateDistance(_taste_balance, permutationBalance);
      if (distance < minDistance) { // if the distance is less than the minimum distance, then update the minimum distance and the suggested menu
        minDistance = distance;
        suggested_menu = std::make_shared<Menu::Menu>(permutation->getMenu());
      }
      else if (distance == minDistance) { // if the distance is equal, then compare the covariances
        double currentCovariance = calculateCovariance(_taste_balance, permutationBalance);
        double suggestedCovariance = calculateCovariance(_taste_balance, suggested_menu->getTasteBalance());
        if (currentCovariance < suggestedCovariance) {
          suggested_menu = std::make_shared<Menu::Menu>(permutation->getMenu());
        } else {
          continue;
        }
      }
    }

    return suggested_menu;
  }
} // End namespace Interface