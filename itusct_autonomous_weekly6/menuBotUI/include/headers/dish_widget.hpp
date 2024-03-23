#ifndef DISH_WIDGET_HPP
#define DISH_WIDGET_HPP

#include <QWidget>
#include <QLabel>
#include <QCheckBox>
#include <QGridLayout>
#include <QDebug>
#include "../menu_bot/interface.hpp"

class DishWidget : public QWidget
{
    Q_OBJECT
public:
    explicit DishWidget(std::shared_ptr<Menu::MenuItem> menuItem, Menu::DishType dishType, QVector<QVector<QCheckBox*>> &checkboxMatrix, QWidget *parent = nullptr);

private:
    QLabel *dishLabel;
    QCheckBox *checkbox1; // at most 2 checkbox is needed
    QCheckBox *checkbox2;

    QGridLayout *layout;
};

#endif // DISH_WIDGET_HPP
