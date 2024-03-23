/**
  ITU Solar Car Team - Autonomous Weekly Assignment 5
  Author: Yusuf Yıldız
  Date: 12.02.2024
*/

#ifndef MENU_BOT_INTERFACE_HPP
#define MENU_BOT_INTERFACE_HPP

#include "menu.hpp"
#include "user.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <climits>

/**
 * @brief Interface namespace contains the functions that are used to interact with the user and the menu.
 */
namespace Interface{
  /**
   * @brief Reads the menu from a JSON file and stores it in a 2D vector of shared pointers to MenuItems.
   * @param _file_name The name of the JSON file to read the menu from.
   * @param _menu The 2D vector of shared pointers to MenuItems to store the menu in.
   */
  void readFromJSON(std::string _file_name, std::vector<std::vector<std::shared_ptr<Menu::MenuItem>>>& _menu);

  /**
   * @brief Calculate mean square distance between two taste balances.
   * @param _balance1 The first taste balance to compare.
   * @param _balance2 The second taste balance to compare.
   * @return The mean square distance between the two taste balances.
   */
  double calculateDistance(const Menu::TasteBalance& _balance1, const Menu::TasteBalance& _balance2);

  /**
   * @brief Calculate covariance between two taste balances.
   * @param _balance1 The first taste balance to compare.
   * @param _balance2 The second taste balance to compare.
   * @return The covariance between the two taste balances.
   */
  double calculateCovariance(const Menu::TasteBalance& _balance1, const Menu::TasteBalance& _balance2);

  /**
   * @brief Recursively generates all permutations of a given vector of integers to be used for menu permutations later on.
   * @param _permutations 2D vector of integers to store the all permutations.
   * @param _menu The 2D vector of shared pointers to MenuItems to write to the JSON file.
   */
  void generatePermutations(std::vector<std::vector<int>>& _permutations, std::vector<int>& _indices, int _level, const std::vector<int>& _max_sizes);

  /**
   * @brief Generates all possible permutations of the available menu.
   *
   * It generates all possible permutations of the available menu of the restaurant and stores them in a vector of shared pointers to Menu.
   *
   * @param _menu The available menu of the restaurant to generate permutations from.
   * @return A vector of shared pointers to Menu containing all possible permutations of the available menu.
   */
  std::vector<std::shared_ptr<Menu::Menu>> generateMenuItemPermutations(std::shared_ptr<Menu::Menu>& _menu);

  /**
   * @brief It chooses the best fit item of type that the user desired.
   * @param _available_menu The available menu of the restaurant, it is used to reach all possible items in the menu.
   * @param _taste_balance The desired taste balance of the user.
   * @param _type The desired type of the dish to suggest.
   * @return A shared pointer to the suggested menu item.
   */
  std::shared_ptr<Menu::MenuItem> suggestMenuItem(std::shared_ptr<Menu::Menu>& _available_menu, Menu::TasteBalance _taste_balance, Menu::DishType _type);

  /**
   * @brief It chooses the best fit menu that contains one menu item of all different dish types.
   *
   * It chooses the best fit menu to desired taste balance,
   * using mean square distance and covariances between all different menu permutations and desired taste balance.
   *
   * @param _available_menu The available menu of the restaurant, it is used to reach all possible items in the menu.
   * @param _taste_balance The desired taste balance of the user.
   * @return A shared pointer to the suggested menu.
   */
  std::shared_ptr<Menu::Menu> suggestFullMenu(std::shared_ptr<Menu::Menu>& _available_menu, Menu::TasteBalance _taste_balance);

} // End namespace Interface

#endif //MENU_BOT_INTERFACE_HPP
