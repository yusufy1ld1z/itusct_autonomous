/**
  ITU Solar Car Team - Autonomous Weekly Assignment 5
  Author: Yusuf Yıldız
  Date: 12.02.2024
*/

#include <menu.hpp>
#include <interface.hpp>
#include <cstdlib>

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
  std::shared_ptr<Menu::Menu> available_menu = std::make_shared<Menu::Menu>(availableMenu);

  auto is_numeric = [](const std::string &str) -> bool { // check whether the string is numeric or not, this function also controls the negative numbers
    return !str.empty() && std::find_if(str.begin(),
                                        str.end(), [](unsigned char c) { return !std::isdigit(c); }) == str.end(); // control the string digit by digit to be numeric
  };

  std::cout << "Welcome to the Menu Bot!" << std::endl;
  std::cout << "Please enter your first name: ";
  std::string firstName;
  std::cin >> firstName;
  std::cout << "Please enter your last name: ";
  std::string lastName;
  std::cin >> lastName;
  std::cout << "Please select your gender to address you by: \n1. MR\n2. MRS" << std::endl;
  int gender_choice;
  User::Gender genderToAddress = User::Gender::MR;
  std::string gender_str;
  std::cin >> gender_str;

  if (is_numeric(gender_str)){ // input validation
    gender_choice = std::stoi(gender_str);

    if (gender_choice == 1) genderToAddress = User::Gender::MR;
    else if (gender_choice == 2) genderToAddress = User::Gender::MRS;
    else {std::cout << "Invalid input. Default value (Mr) assigned" << std::endl;}
    sleep(1);

  } else {
    std::cout << "Invalid input. Default value (Mr) assigned" << std::endl;
    sleep(1);
  }

  User::User user(firstName, lastName, genderToAddress);
  sleep(1);

  while(true) {
    system("clear");
    std::cout << "Welcome " << (user.getGender() == User::Gender::MR ? "Mr. " : "Mrs. ") << user.getFirstName()
              << " " << user.getLastName() << std::endl;

    std::cout << "What would you like to do?" << std::endl;
    std::cout << "1. See our menu" << std::endl;
    std::cout << "2. Add item to your menu" << std::endl;
    std::cout << "3. Remove item from your menu" << std::endl;
    std::cout << "4. Approve your menu" << std::endl;
    std::cout << "5. Print your menu" << std::endl;
    std::cout << "6. Get menu item suggestion" << std::endl;
    std::cout << "7. Get full menu suggestion" << std::endl;
    std::cout << "8. Exit" << std::endl;
    std::cout << "Please enter your choice: ";

    std::string choice_str;
    std::cin >> choice_str;
    int choice;

    if (is_numeric(choice_str)){ // input validation
      if (std::stoi(choice_str) < 1 || std::stoi(choice_str) > 8){
        std::cout << "Invalid input. Please try again." << std::endl;
        sleep(1);
        continue;
      }
      choice = std::stoi(choice_str);
    } else {
      std::cout << "Invalid input. Please try again." << std::endl;
      sleep(1);
      continue;
    }

    switch (choice){
      case 1: { // print available menu
        system("clear");
        available_menu->printMenu();
        std::cout << "Press enter to continue" << std::endl;
        std::cin.ignore();
        std::cin.get();
        break;
      }
      case 2: { // add item to user's menu
        system("clear");
        std::cout << "Please enter the type of the item you want to add: \n1. Starter\n2. Salad\n3. Main Course\n4. Drink\n5. Appetizer\n6. Dessert\n7. Return to Main Page" << std::endl;
        std::string type_str;
        std::cin >> type_str;
        int type;

        if (is_numeric(type_str)){ // input validation
          if (std::stoi(type_str) < 1 || std::stoi(type_str) > 7){
            std::cout << "Invalid input. Please try again." << std::endl;
            sleep(1);
            continue;
          }
          type = std::stoi(type_str);
        } else {
          std::cout << "Invalid input. Please try again." << std::endl;
          sleep(1);
          continue;
        }

        if (type == 1){
          if(Interface::addItemByType(user, available_menu, Menu::DishType::STARTER)){
            std::cout << "Item added successfully!" << std::endl;
          } else {
            std::cout << "Item did not added!" << std::endl;
          }
          sleep(1);
        }
        else if (type == 2){
          if(Interface::addItemByType(user, available_menu, Menu::DishType::SALAD)){
            std::cout << "Item added successfully!" << std::endl;
          } else {
            std::cout << "Item did not added!" << std::endl;
          }
          sleep(1);
        }
        else if (type == 3){
          if(Interface::addItemByType(user, available_menu, Menu::DishType::MAIN_COURSE)){
            std::cout << "Item added successfully!" << std::endl;
          } else {
            std::cout << "Item did not added!" << std::endl;
          }
          sleep(1);
        }
        else if (type == 4){
          if(Interface::addItemByType(user, available_menu, Menu::DishType::DRINK)){
            std::cout << "Item added successfully!" << std::endl;
          } else {
            std::cout << "Item did not added!" << std::endl;
          }
          sleep(1);
        }
        else if (type == 5){
          if(Interface::addItemByType(user, available_menu, Menu::DishType::APPETIZER)){
            std::cout << "Item added successfully!" << std::endl;
          } else {
            std::cout << "Item did not added!" << std::endl;
          }
          sleep(1);
        }
        else if (type == 6){
          if(Interface::addItemByType(user, available_menu, Menu::DishType::DESSERT)){
            std::cout << "Item added successfully!" << std::endl;
          } else {
            std::cout << "Item did not added!" << std::endl;
          }
          sleep(1);
        }
        else if (type == 7){
          std::cout << "Returning to main page..." << std::endl;
          sleep(1);
          continue;
        }
        else {
          std::cout << "Invalid input. Please try again." << std::endl;
          sleep(1);
          continue;
        }
        break;
      }
      case 3: { // remove item from user's menu
        int index = 0;
        std::cout << "++++++++++++++++++++" << std::endl;
        for (int i = 0; i < static_cast<int>(Menu::DishType::NUM_DISH_TYPES); i++) {
          std::string type;
          switch (i) {
            case Menu::DishType::STARTER:
              type = "Starters";
              break;
            case Menu::DishType::SALAD:
              type = "Salads";
              break;
            case Menu::DishType::MAIN_COURSE:
              type = "Main Courses";
              break;
            case Menu::DishType::DRINK:
              type = "Drinks";
              break;
            case Menu::DishType::APPETIZER:
              type = "Appetizers";
              break;
            case Menu::DishType::DESSERT:
              type = "Desserts";
              break;
            default:
              break;
          }

          if(user.getMenu()->getMenu()[i].empty()){
            std::cout << "Empty Preference for Dish Type: " << type << std::endl;
          } else {
            std::cout << "----------------" << std::endl;
            for (auto &item: user.getMenu()->getMenu()[i]) {
              std::cout << ++index << ". ";
              item->printDish();
              std::cout << "++++++++++++++++++++" << std::endl;
            }
          }
        }
        if (index == 0){
          std::cout << "Your menu is empty!" << std::endl;
          sleep(2);
          continue;
        }
        std::cout << "Which item would you like to remove?" << std::endl;
        std::string remove_str;
        std::cin >> remove_str;
        int remove_choice;

        if (is_numeric(remove_str)){ // input validation
          if (std::stoi(remove_str) < 1 || std::stoi(remove_str) > 7){
            std::cout << "Invalid input. Please try again." << std::endl;
            sleep(1);
            continue;
          }
          remove_choice = std::stoi(remove_str);
        } else {
          std::cout << "Invalid input. Please try again." << std::endl;
          sleep(1);
          continue;
        }

        std::cout << "Chosen item is: " << std::endl;
        user.getMenu()->findMenuItemByIndex(remove_choice)->printDish();
        std::cout << "\nAre you sure to remove this item?(Y or N)" << std::endl;
        char confirm;
        std::cin >> confirm;

        if (confirm == 'Y' || confirm == 'y') {
          user.getMenu()->removeMenuItem(user.getMenu()->findMenuItemByIndex(remove_choice)->getName());
          std::cout << "Item removed successfully!" << std::endl;
        } else {
          std::cout << "Item did not removed!" << std::endl;
          sleep(1);
        }
        break;
      }

      case 4: // approve user's menu
        system("clear");
        std::cout << "Before your menu is served, we must ask your preferences and whether you want some extra services." << std::endl;

        if (user.approveMenu()){
          std::cout << "Your menu is approved!" << std::endl;
          std::cout << "Thanks for using our menu bot, we hope you enjoyed. You may pay your bill to the cashier!" << std::endl;
          std::cout << "Goodbye!" << std::endl;
          sleep(1);
          return 0;
        } else {
          std::cout << "Your menu is not approved!" << std::endl;
        }
        break;

      case 5: { // print user's menu
        system("clear");
        user.getMenu()->printMenu();
        std::cout << "Press enter to continue" << std::endl;
        std::cin.ignore();
        std::cin.get();
        break;
      }

      case 6: { // get menu item suggestion
        std::cout << "Menu item suggestion gives you the best fit menu item to your desired taste balance!";
        std::cout << "Please enter the type of the item you want to get suggestion for: \n1. Starter\n2. Salad\n3. Main Course\n4. Drink\n5. Appetizer\n6. Dessert" << std::endl;
        int type;
        std::string type_str;

        std::cin >> type_str;

        if (is_numeric(type_str)){ // input validation
          if (std::stoi(type_str) < 1 || std::stoi(type_str) > 7){
            std::cout << "Invalid input. Please try again." << std::endl;
            sleep(1);
            continue;
          }
          type = std::stoi(type_str);
        } else {
          std::cout << "Invalid input. Please try again." << std::endl;
          sleep(1);
          continue;
        }

        std::cout << "Please enter your desired taste balance in the order of: Sweet, Sour, Salty, Bitter, Savory: " << std::endl;
        std::vector<int> taste_balance(static_cast<int>(Menu::Taste::NUM_TASTES));
        int validator = 0;
        for (int i = 0; i < static_cast<int>(Menu::Taste::NUM_TASTES); i++) {
          Menu::Taste taste = static_cast<Menu::Taste>(i);
          switch (taste) {
            case Menu::Taste::SWEET:
              std::cout << "Sweet: ";
              break;
            case Menu::Taste::SOUR:
              std::cout << "Sour: ";
              break;
            case Menu::Taste::SALTY:
              std::cout << "Salty: ";
              break;
            case Menu::Taste::BITTER:
              std::cout << "Bitter: ";
              break;
            case Menu::Taste::SAVORY:
              std::cout << "Savory: ";
              break;
            default:
              break;
          }
          std::string taste_str;
          std::cin >> taste_str;

          if (is_numeric(taste_str)){ // input validation
            if (std::stoi(taste_str) < 0 || std::stoi(taste_str) > 10){
              std::cout << "Invalid input. Please try again." << std::endl;
              sleep(1);
              break;
            }
            validator++;
            taste_balance[i] = std::stoi(taste_str);
          } else {
            std::cout << "Invalid input. Please try again." << std::endl;
            sleep(1);
            break;
          }
        }

        if (validator != static_cast<int>(Menu::Taste::NUM_TASTES)) continue;

        std::shared_ptr<Menu::MenuItem> suggested_menu_item = Interface::suggestMenuItem(available_menu, Menu::TasteBalance(taste_balance), static_cast<Menu::DishType>(type - 1));
        suggested_menu_item->printDish();

        std::cout << "\nAre you sure to choose this suggested menu item?(Y or N)" << std::endl;
        char confirm;
        std::cin >> confirm;

        if (confirm == 'Y' || confirm == 'y') {
          user.getMenu()->addMenuItem(suggested_menu_item);
          std::cout << "Suggested menu item is chosen successfully!" << std::endl;
        } else {
          std::cout << "Suggested menu item is not chosen, for another suggestion, please try again!" << std::endl;
        }
        sleep(1);
        break;
      }

      case 7: { // get full menu suggestion
        std::cout << "Full menu suggestion gives you the best fit full menu to your desired taste balance!";
        std::cout << "Please enter your desired taste balance in the order of: Sweet, Sour, Salty, Bitter, Savory: " << std::endl;
        std::vector<int> taste_balance(static_cast<int>(Menu::Taste::NUM_TASTES));
        int validator = 0;
        for (int i = 0; i < static_cast<int>(Menu::Taste::NUM_TASTES); i++) {
          Menu::Taste taste = static_cast<Menu::Taste>(i);
          switch (taste) {
            case Menu::Taste::SWEET:
              std::cout << "Sweet: ";
              break;
            case Menu::Taste::SOUR:
              std::cout << "Sour: ";
              break;
            case Menu::Taste::SALTY:
              std::cout << "Salty: ";
              break;
            case Menu::Taste::BITTER:
              std::cout << "Bitter: ";
              break;
            case Menu::Taste::SAVORY:
              std::cout << "Savory: ";
              break;
          }

          std::string taste_str;
          std::cin >> taste_str;

          if (is_numeric(taste_str)){ // input validation
            if (std::stoi(taste_str) < 0 || std::stoi(taste_str) > 10){
              std::cout << "Invalid input. Please try again." << std::endl;
              sleep(1);
              break;
            }
            validator++;
            taste_balance[i] = std::stoi(taste_str);
          } else {
            std::cout << "Invalid input. Please try again." << std::endl;
            sleep(1);
            break;
          }
        }

        if (validator != static_cast<int>(Menu::Taste::NUM_TASTES)) continue;

        std::shared_ptr<Menu::Menu> suggested_menu = Interface::suggestFullMenu(available_menu, Menu::TasteBalance(taste_balance));
        suggested_menu->printMenu();
        std::cout << "Your desired taste balance: " << std::endl;
        Menu::TasteBalance(taste_balance).print();

        std::cout << "\nAre you sure to choose this suggested menu?(Y or N)" << std::endl;
        char confirm;
        std::cin >> confirm;

        if (confirm == 'Y' || confirm == 'y') {
          for (int i = 0; i < static_cast<int>(Menu::DishType::NUM_DISH_TYPES); i++) {
            for (auto &item: suggested_menu->getMenu()[i]) {
              user.getMenu()->addMenuItem(item);
            }
          }
          std::cout << "Suggested menu is chosen successfully!" << std::endl;
        } else {
          std::cout << "Suggested menu is not chosen, for another suggestion, please try again!" << std::endl;
        }
        sleep(1);
        break;
      }

      case 8: { // exit
        std::cout << "Goodbye!" << std::endl;
        return 0;
      }

      default: {
        std::cout << "Invalid input. Please try again." << std::endl;
        sleep(1);
        break;
      }
    }

  }
}