/**
  ITU Solar Car Team - Autonomous Weekly Assignment 5
  Author: Yusuf Yıldız
  Date: 12.02.2024
*/

#include <menu.hpp>
#include <interface.hpp>

void TEST_addMenuItem() {
  std::string file_name = "/home/yildiz/Desktop/gae/autonomous_assignments/itusct_autonomous_weekly5/assignment5_solutions/menuBot/menu.json";
  std::vector<std::vector<std::shared_ptr<Menu::MenuItem>>> availableMenu(6);
  try {
    Interface::readFromJSON(file_name, availableMenu);
  }
  catch (const std::exception& e) {
    std::cout << "Cant parse JSON. Error: " << e.what() << std::endl;
    return;
  }

  std::cout << "TEST MENU" << std::endl;
  Menu::Menu menu(availableMenu);

  std::shared_ptr<Menu::MenuItem> testStarter = std::make_shared<Menu::Starter>("testStarter", 10, Menu::TasteBalance(1, 2, 3, 4, 5));
  std::cout << "TEST STARTER" << std::endl;
  menu.addMenuItem(testStarter);

  for (auto& item: availableMenu[Menu::STARTER]) {
    std::cout << "---------------" << std::endl;
    item->printDish();
  }

  menu.getMenu()[Menu::STARTER][5]->printDish();
}

void TEST_printMenu(){
  std::string file_name = "/home/yildiz/Desktop/gae/autonomous_assignments/itusct_autonomous_weekly5/assignment5_solutions/menuBot/menu.json";
  std::vector<std::vector<std::shared_ptr<Menu::MenuItem>>> availableMenu(6);
  try {
    Interface::readFromJSON(file_name, availableMenu);
  }
  catch (const std::exception& e) {
    std::cout << "Cant parse JSON. Error: " << e.what() << std::endl;
    return;
  }

  std::cout << "TEST MENU" << std::endl;
  Menu::Menu menu(availableMenu);
  std::cout << "-----------Menu read from JSON-----------" << std::endl;
  menu.printMenu();

}

void TEST_printSubMenu(){
  std::string file_name = "/home/yildiz/Desktop/gae/autonomous_assignments/itusct_autonomous_weekly5/assignment5_solutions/menuBot/menu.json";
  std::vector<std::vector<std::shared_ptr<Menu::MenuItem>>> availableMenu(6);
  try {
    Interface::readFromJSON(file_name, availableMenu);
  }
  catch (const std::exception& e) {
    std::cout << "Cant parse JSON. Error: " << e.what() << std::endl;
    return;
  }
  std::cout << "TEST MENU" << std::endl;
  Menu::Menu menu(availableMenu);
  std::cout << "-----------Sub Menu read from JSON-----------" << std::endl;
  int index = 1;
  for (auto& item: menu.getMenu()[0]) {
    std::cout << index++ << ". ";
    item->printDish();
  }
  std::cout << "END" << std::endl;
}

int main() {
  // TEST_addMenuItem();
  // TEST_printMenu();
  TEST_printSubMenu();
  return 0;
}