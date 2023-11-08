#include"calculator.hpp"

template <typename T>
void function(Calculator<T> calculator);

int main(){
    
    std::cout << "First you must enter the type you want to use the calculator with:" << std::endl
    << "1. int" << std::endl
    << "2. float" << std::endl
    << "3. double" << std::endl;
    int choice;
    std::cin >> choice;
    Calculator<int> myIntCalculator;
    Calculator<float> myFloatCalculator;
    Calculator<double> myDoubleCalculator;
    switch(choice){
        case 1:{
            std::cout << "Integer Calculator choosen!" << std::endl;
            function(myIntCalculator);
            break;
        }
        case 2:{
            std::cout << "Float Calculator choosen!" << std::endl;
            function(myFloatCalculator);
            break;
        }
        case 3:{
            std::cout << "Double Calculator choosen!" << std::endl;
            function(myDoubleCalculator);
            break;
        }
        default:{
            std::cout << "Invalid choice!" << std::endl;
            break;
        }
    }

    return 0;
}

template <typename T>
void function(Calculator<T> calculator){
    std::cout << "Welcome to the calculator!" << std::endl;
    std::cout << "Operations you can make: " << std::endl
    << "1. Addition" << std::endl
    << "2. Subtraction" << std::endl
    << "3. Multiplication" << std::endl
    << "4. Division" << std::endl
    << "5. Square" << std::endl
    << "6. Exponent" << std::endl
    << "7. Modulus" << std::endl;

    bool on = true;
    int operation;
    while(on){
        std::cout << "Enter the operation you want to make: ";
        std::cin >> operation;
        std::cout << "Enter the operand(s): " << std::endl;
        switch(operation){
            case 1:{
                T a; T b;
                std::cin >> a >> b;
                std::cout << "The result is: " << calculator.add(a, b) << std::endl;
                break;
            }
            case 2:{
                T a; T b;
                std::cin >> a >> b;
                std::cout << "The result is: " << calculator.subtract(a, b) << std::endl;
                break;
            }
            case 3:{
                T a; T b;
                std::cin >> a >> b;
                std::cout << "The result is: " << calculator.multiply(a, b) << std::endl;
                break;
            }
            case 4:{
                T a; T b;
                std::cin >> a >> b;
                try{
                    std::cout << "The result is: " << calculator.divide(a, b) << std::endl;
                }
                catch(std::invalid_argument& e){
                    std::cout << e.what() << std::endl;
                }
                break;
            }
            case 5:{
                T a;
                std::cin >> a;
                std::cout << "The result is: " << calculator.square(a) << std::endl;
                break;
            }
            case 6:{
                T a; T b;
                std::cin >> a >> b;
                std::cout << "The result is: " << calculator.exp(a, b) << std::endl;
                break;
            }
            case 7:{
                T a; T b;
                std::cin >> a >> b;
                try{
                    std::cout << "The result is: " << calculator.mod(a, b) << std::endl;
                }
                catch(std::invalid_argument& e){
                    std::cout << e.what() << std::endl;
                }
                break;
            }
            default:{
                std::cout << "Invalid choice!" << std::endl;
                break;
            }
        }
        std::cout << "Do you want to continue? (y/n): ";
        char choice;
        std::cin >> choice;
        if (choice == 'n'){
            on = false;
            std::cout << "Thank you for using the calculator!" << std::endl;
        };
        std::cout << "-----------------------------------------" << std::endl;
    }
}