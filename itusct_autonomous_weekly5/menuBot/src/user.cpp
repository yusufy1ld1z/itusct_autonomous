//
// Created by yildiz on 10.02.2024.
//

#include "user.hpp"

namespace User{
  User::User(std::string _first_name, std::string _last_name, Gender _gender = Gender::MR) {
    setFirstName(_first_name);
    setLastName(_last_name);
    setGender(_gender);
    user_menu = std::make_shared<Menu::Menu>();
  }
// TODO: preference'lar degistirilebilir olmali, olumsuz secenekler icin de set et
  bool User::approveMenu() {
    char confirm;
    for (int i = 0; i < static_cast<int>(Menu::DishType::NUM_DISH_TYPES); i++) {
      Menu::DishType type = static_cast<Menu::DishType>(i);
      if (!this->getMenu()->getMenu()[i].empty()) {
        for (auto &item: this->getMenu()->getMenu()[i]) {
          switch (type) {
            case Menu::DishType::STARTER:
              std::cout << "----------------" << std::endl;
              item->printDish();
              std::cout << "----------------" << std::endl;

              std::cout << "Do you want us to serve your starter as hot? (Y or N)" << std::endl;

              std::cin >> confirm;

              if (confirm == 'Y' || confirm == 'y') {
                // polymorphism de olsa sadece child'a ozgu methodlar icin mutlaka downcast yapilmali
                if(std::shared_ptr<Menu::Starter> starterPtr = std::dynamic_pointer_cast<Menu::Starter>(item)) starterPtr->setIsHot(true);
                std::cout << "Your starter will be served as hot!" << std::endl;
              } else {
                if(std::shared_ptr<Menu::Starter> starterPtr = std::dynamic_pointer_cast<Menu::Starter>(item)) starterPtr->setIsHot(false);
                std::cout << "Your starter will be served as cold!" << std::endl;
                sleep(1);
              }
              break;
            case Menu::DishType::SALAD:
              std::cout << "----------------" << std::endl;
              item->printDish();
              std::cout << "----------------" << std::endl;

              std::cout << "Do you want an extra topping to your salad with an extra cost $2.25? (Y or N)" << std::endl;

              std::cin >> confirm;

              if (confirm == 'Y' || confirm == 'y') {
                std::cout << "Please enter the topping you want to add: ";
                std::string topping;
                std::cin >> topping;

                if(std::shared_ptr<Menu::Salads> saladPtr = std::dynamic_pointer_cast<Menu::Salads>(item)) {
                  saladPtr->setAddTopping(true);
                  saladPtr->setTopping(topping);

                }
                this->getMenu()->setTotalPrice(this->getMenu()->getTotalPrice() + Menu::Extras::salad_topping);
                std::cout << "Your salad will be served with an extra topping " << topping << "!" << std::endl;
              } else {
                if(std::shared_ptr<Menu::Salads> saladPtr = std::dynamic_pointer_cast<Menu::Salads>(item)) {
                  if (saladPtr->getAddTopping()) this->getMenu()->setTotalPrice(this->getMenu()->getTotalPrice() - Menu::Extras::salad_topping);
                  saladPtr->setAddTopping(false);
                  saladPtr->setTopping("");
                }
                std::cout << "Your salad will not be served with an extra topping!" << std::endl;
                sleep(1);
              }
              break;
            case Menu::DishType::MAIN_COURSE:
              std::cout << "----------------" << std::endl;
              item->printDish();
              std::cout << "----------------" << std::endl;

              std::cout << "Do you want us to serve your main course as vegan option? (Y or N)" << std::endl;

              std::cin >> confirm;

              if (confirm == 'Y' || confirm == 'y') {
                if(std::shared_ptr<Menu::MainCourse> main_coursePtr = std::dynamic_pointer_cast<Menu::MainCourse>(item)) main_coursePtr->setIsVegan(true);
                std::cout << "Your main course will be served with a vegan option!" << std::endl;
              } else {
                if(std::shared_ptr<Menu::MainCourse> main_coursePtr = std::dynamic_pointer_cast<Menu::MainCourse>(item)) main_coursePtr->setIsVegan(false);
                std::cout << "Your main course will not be served with a vegan option!" << std::endl;
                sleep(1);
              }
              break;
            case Menu::DishType::DRINK:
              std::cout << "----------------" << std::endl;
              item->printDish();
              std::cout << "----------------" << std::endl;

              std::cout << "Do you want to add some extra carbonation to your drink with an extra cost $0.5? (Y or N)" << std::endl;

              std::cin >> confirm;

              if (confirm == 'Y' || confirm == 'y') {
                if(std::shared_ptr<Menu::Drinks> drinkPtr = std::dynamic_pointer_cast<Menu::Drinks>(item)) drinkPtr->setIsCarbonated(true);
                this->getMenu()->setTotalPrice(this->getMenu()->getTotalPrice() + Menu::Extras::drink_carbonation);

                std::cout << "Your drink will be served with an extra carbonation!" << std::endl;
              } else {
                if(std::shared_ptr<Menu::Drinks> drinkPtr = std::dynamic_pointer_cast<Menu::Drinks>(item)){
                  if (drinkPtr->getIsCarbonated()) this->getMenu()->setTotalPrice(this->getMenu()->getTotalPrice() - Menu::Extras::drink_carbonation);
                  drinkPtr->setIsCarbonated(false);
                }
                std::cout << "Your drink will not be served with an extra carbonation!" << std::endl;
                sleep(1);
              }

              std::cout << "Do you want to add some extra alcohol shot to your drink with an extra cost $2.5? (Y or N)" << std::endl;

              std::cin >> confirm;

              if (confirm == 'Y' || confirm == 'y') {
                if(std::shared_ptr<Menu::Drinks> drinkPtr = std::dynamic_pointer_cast<Menu::Drinks>(item)) drinkPtr->setIsAlcoholic(true);
                this->getMenu()->setTotalPrice(this->getMenu()->getTotalPrice() + Menu::Extras::drink_alcohol);

                std::cout << "Your drink will be served with an extra alcohol shot!" << std::endl;
              } else {
                if(std::shared_ptr<Menu::Drinks> drinkPtr = std::dynamic_pointer_cast<Menu::Drinks>(item)){
                  if (drinkPtr->getIsAlcoholic()) this->getMenu()->setTotalPrice(this->getMenu()->getTotalPrice() - Menu::Extras::drink_alcohol);
                  drinkPtr->setIsAlcoholic(false);
                }
                std::cout << "Your drink will not be served with an extra alcohol shot!" << std::endl;
                sleep(1);
              }
              break;
            case Menu::DishType::APPETIZER:
              std::cout << "----------------" << std::endl;
              item->printDish();
              std::cout << "----------------" << std::endl;

              std::cout << "Do you want us to serve your appetizer before the main course, as a tradition we serve them after the main course? (Y or N)" << std::endl;

              std::cin >> confirm;

              if (confirm == 'Y' || confirm == 'y') {
                if(std::shared_ptr<Menu::Appetizer> appetizerPtr = std::dynamic_pointer_cast<Menu::Appetizer>(item)) appetizerPtr->setIsBeforeMainCourse(true);
                std::cout << "Your appetizer will be served before your main course!" << std::endl;
              } else {
                if(std::shared_ptr<Menu::Appetizer> appetizerPtr = std::dynamic_pointer_cast<Menu::Appetizer>(item)) appetizerPtr->setIsBeforeMainCourse(false);
                std::cout << "Your appetizer will be served after your main course!" << std::endl;
                sleep(1);
              }
              break;
            case Menu::DishType::DESSERT:
              std::cout << "----------------" << std::endl;
              item->printDish();
              std::cout << "----------------" << std::endl;

              std::cout << "Do you want to add some extra chocolate to your dessert with an extra cost $1.5? (Y or N)" << std::endl;

              std::cin >> confirm;

              if (confirm == 'Y' || confirm == 'y') {
                if(std::shared_ptr<Menu::Desserts> dessertPtr = std::dynamic_pointer_cast<Menu::Desserts>(item)) dessertPtr->setAddChocolate(true);

                this->getMenu()->setTotalPrice(this->getMenu()->getTotalPrice() + Menu::Extras::dessert_chocolate);

                std::cout << "Your dessert will be served with an extra chocolate!" << std::endl;
              } else {
                if(std::shared_ptr<Menu::Desserts> dessertPtr = std::dynamic_pointer_cast<Menu::Desserts>(item)){
                  if (dessertPtr->getAddChocolate()) this->getMenu()->setTotalPrice(this->getMenu()->getTotalPrice() - Menu::Extras::dessert_chocolate);
                  dessertPtr->setAddChocolate(false);
                }
                std::cout << "Your dessert will not be served with an extra chocolate!" << std::endl;
                sleep(1);
              }
          }
        }
      }
    }
    std::cout << "Do you want to see your last menu before approval? (Y or N)" << std::endl;

    std::cin >> confirm;

    if (confirm == 'Y' || confirm == 'y') {
      this->getMenu()->printMenu();
      std::cout << "Press enter to continue" << std::endl;
      std::cin.ignore();
      std::cin.get();
    }

    std::cout << "Do you want to approve your menu? (Y or N)" << std::endl;
    std::cin >> confirm;

    if (confirm == 'Y' || confirm == 'y') {
      return true;
    } else {
      return false;
    }
  }
} // End namespace User