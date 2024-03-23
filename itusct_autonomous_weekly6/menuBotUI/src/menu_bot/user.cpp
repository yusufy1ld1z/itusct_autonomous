/**
  ITU Solar Car Team - Autonomous Weekly Assignment 5
  Author: Yusuf Yıldız
  Date: 12.02.2024
*/

#include "../../include/menu_bot/user.hpp"

namespace User{
  User::User(std::string _first_name, std::string _last_name, Gender _gender = Gender::MR) {
    setFirstName(_first_name);
    setLastName(_last_name);
    setGender(_gender);
    user_menu = std::make_shared<Menu::Menu>();
  }

  User::User(){
    first_name = "";
    last_name = "";
    gender_to_address = Gender::MR;
    user_menu = std::make_shared<Menu::Menu>();
  }
} // End namespace User
