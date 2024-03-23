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

    /**
     * @brief It creates widgets according to the dish type of the items in the user menu, then adds it to the vertical layout.
     */
    void addDishWidgets();

    ~approveMenu();

signals:
    void return_userMenu();

private slots:
    /**
     * @brief It updates the additional preferences according to the checkbox status, and then return back to user menu window.
     */
    void on_pushButton_approve_clicked();

    /**
     * @brief Emitter for returning back to the user menu.
     */
    void on_pushButton_return_clicked();

private:
    Ui::approveMenu *ui;
    User::User *user;

    QVector<QVector<QCheckBox*>> checkboxMatrix; // to store the preference checkboxes for every dish type
};

#endif // APPROVE_MENU_HPP
