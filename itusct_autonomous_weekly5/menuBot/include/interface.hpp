//
// Created by yildiz on 07.02.2024.
//

#ifndef MENU_BOT_INTERFACE_HPP
#define MENU_BOT_INTERFACE_HPP

#include <menu.hpp>
#include <user.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <climits>


namespace Interface{
  void readFromJSON(std::string _file_name, std::vector<std::vector<std::shared_ptr<Menu::MenuItem>>>& _menu);
  bool addItemByType(User::User& _user, std::shared_ptr<Menu::Menu>& _available_menu, Menu::DishType _type);
  double calculateDistance(const Menu::TasteBalance& _balance1, const Menu::TasteBalance& _balance2);
  double calculateCovariance(const Menu::TasteBalance& _balance1, const Menu::TasteBalance& _balance2);
  void generatePermutations(std::vector<std::vector<int>>& _permutations, std::vector<int>& _indices, int _level, const std::vector<int>& _max_sizes);
  std::vector<std::shared_ptr<Menu::Menu>> generateMenuItemPermutations(std::vector<std::shared_ptr<Menu::MenuItem>>& _menu);
  std::shared_ptr<Menu::MenuItem> suggestMenuItem(std::shared_ptr<Menu::Menu>& _available_menu, Menu::TasteBalance _taste_balance, Menu::DishType _type);
  std::shared_ptr<Menu::Menu> suggestFullMenu(std::shared_ptr<Menu::Menu>& _available_menu, Menu::TasteBalance _taste_balance);

} // End namespace Interface

#endif //MENU_BOT_INTERFACE_HPP