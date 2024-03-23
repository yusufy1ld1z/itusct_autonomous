QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    ./src/menu_bot/interface.cpp \
    ./src/menu_bot/menu.cpp \
    ./src/menu_bot/user.cpp \
    ./src/sources/approve_menu.cpp \
    ./src/sources/dish_widget.cpp \
    ./src/sources/login_screen.cpp \
    ./src/sources/main.cpp \
    ./src/sources/main_screen.cpp \
    ./src/sources/show_items.cpp \
    ./src/sources/show_menu.cpp \
    ./src/sources/suggestion_screen.cpp

HEADERS += \
    ./include/menu_bot/interface.hpp \
    ./include/menu_bot/menu.hpp \
    ./include/menu_bot/user.hpp \
    ./include/headers/approve_menu.hpp \
    ./include/headers/dish_widget.hpp \
    ./include/headers/login_screen.hpp \
    ./include/headers/main_screen.hpp \
    ./include/headers/show_items.hpp \
    ./include/headers/show_menu.hpp \
    ./include/headers/suggestion_screen.hpp

FORMS += \
    ./include/ui/approve_menu.ui \
    ./include/ui/login_screen.ui \
    ./include/ui/main_screen.ui \
    ./include/ui/show_items.ui \
    ./include/ui/show_menu.ui \
    ./include/ui/suggestion_screen.ui

RESOURCES += \
    ./resources/images.qrc

