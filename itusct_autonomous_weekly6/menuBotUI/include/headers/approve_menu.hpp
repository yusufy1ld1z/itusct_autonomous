#ifndef APPROVE_MENU_HPP
#define APPROVE_MENU_HPP

#include <QDialog>
#include <QDebug>
#include <QWidget>
#include <QCheckBox>
#include <QMessageBox>
#include "../menu_bot/interface.hpp"

namespace Ui {
class approveMenu;
}

class approveMenu : public QDialog
{
    Q_OBJECT

public:
    explicit approveMenu(QWidget *parent = nullptr, User::User *_user = nullptr, QString _window_name = "Dialog");

    void addDishWidgets();

    ~approveMenu();

signals:
    void return_userMenu();

private slots:
    void on_pushButton_approve_clicked();

    void on_pushButton_return_clicked();

private:
    Ui::approveMenu *ui;
    User::User *user;

    QVector<QVector<QCheckBox*>> checkboxMatrix;
};

#endif // APPROVE_MENU_HPP
