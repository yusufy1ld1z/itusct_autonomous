#ifndef CALCULATOR_HPP
#define CALCULATOR_HPP

#include<iostream>
#include<cmath>
#include<stdexcept>

template <typename T>
class Calculator
{
    public:
        Calculator(){};
        T add(T a, T b){return a + b;};
        T subtract(T a, T b){return a - b;};
        T multiply(T a, T b){return a * b;};
        T divide(T a, T b){
            if (b == 0)
            {
                throw std::invalid_argument("Cannot divide by zero!");
            }
            else
            {
                return a / b;
            }
        };
        T square(T a){return a * a;};
        T exp(T a, T b){return std::pow(a, b);};
        T mod(T a, T b){
            if (b == 0)
            {
                throw std::invalid_argument("The modulus can not be zero!");
            }
            else
            {   
                if constexpr (std::is_floating_point<T>::value)
                {
                    return std::fmod(a, b);
                }
                else{
                    return a % b;
                }
            }
        };
};

#endif
