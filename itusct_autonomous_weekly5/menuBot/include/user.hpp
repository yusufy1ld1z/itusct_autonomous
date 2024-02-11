//
// Created by yildiz on 10.02.2024.
//

#ifndef MENU_BOT_USER_HPP
#define MENU_BOT_USER_HPP

#include "menu.hpp"
#include <unistd.h>

namespace User{

  enum Gender { MR, MRS };

  class User {
  private:
    std::string first_name;
    std::string last_name;
    Gender gender_to_address;
    std::shared_ptr<Menu::Menu> user_menu;

  public:
    User(std::string _first_name, std::string _last_name, Gender _gender);
    std::string getFirstName() const { return first_name; };
    std::string getLastName() const { return last_name; };
    Gender getGender() const { return gender_to_address; };
    std::shared_ptr<Menu::Menu> getMenu() const { return user_menu; };

    void setMenu(std::shared_ptr<Menu::Menu> _menu) { user_menu = _menu; };
    void setFirstName(std::string _first_name) { first_name = _first_name; };
    void setLastName(std::string _last_name) { last_name = _last_name; };
    void setGender(Gender _gender) { gender_to_address = _gender; }
    bool approveMenu();
  };
} // End namespace User
#endif //MENU_BOT_USER_HPP
