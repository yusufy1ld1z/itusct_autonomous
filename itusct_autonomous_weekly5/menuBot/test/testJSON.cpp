//
// Created by yildiz on 09.02.2024.
//
#include <menu.hpp>
#include <interface.hpp>

int main() {
  std::string file_name = "/home/yildiz/Desktop/gae/autonomous_assignments/itusct_autonomous_weekly5/assignment5_solutions/menuBot/menu.json";
  std::vector<std::vector<std::shared_ptr<Menu::MenuItem>>> availableMenu(6);
  try {
    Interface::readFromJSON(file_name, availableMenu);
  }
  catch (const std::exception& e) {
    std::cout << "Cant parse JSON. Error: " << e.what() << std::endl;
    return 1;
  }

  //  availableMenu[0][0].printDish();
  std::cout << "TEST JSON" << std::endl;
  Menu::Menu menu(availableMenu);
  std::cout << "-----------Menu read from JSON-----------" << std::endl;
  menu.printMenu();

  return 0;
}