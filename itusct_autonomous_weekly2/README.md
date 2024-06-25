# C++ Variables

## Overview

This Project is a simple C++ program that demonstrates the use of variables in C++.
The program displays some properties of variables in C++ such as the **size of the variable**,
the **max** & **min** value of the variable, and the **size of its address** in memory. Project was made
to make it easier for beginners to understand the concept of variables in C++.

## Usage

1. Clone the repository
2. Open the project in your favorite C++ IDE
3. Run the program using Cmake or the IDE's built-in compiler

## Variables
### Used
**C++ char types** - `char`, `unsigned char`, `signed char`, `wchar_t`, `char8_t`, `char16_t`, `char32_t`\
**C++ integer types** - `int`, `signed`, `unsigned`, `short int`, `short`, `long int`, `long`, `long long int`, `signed short int`, `signed int`, `signed long int`, `signed long long int`, `unsigned short int`, `unsigned int`, `unsigned long int`, `unsigned long long int`\
**C++ floating point types** - `float`, `double`, `long double`\
**C++ boolean types** - `bool`\
**C++ built in integer types** - `int8_t`, `uint8_t`, `int16_t`, `uint16_t`, `int32_t`, `uint32_t`, `int64_t`, `uint64_t`
### Unused
**C++ floating point types** - `float16_t`, `float32_t`, `float64_t`, `float128_t`, `bfloat16_t`

## Output

<!-- Table 1: Type Sizes -->
<table>
    <thead>
        <tr>
            <th>Type</th>
            <th>Size</th>
            <th>Pointer Size</th>
            <th>Min</th>
            <th>Max</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td>char</td>
            <td>1</td>
            <td>8</td>
            <td>-128</td>
            <td>127</td>
        </tr>
        <tr>
            <td>unsigned char</td>
            <td>1</td>
            <td>8</td>
            <td>0</td>
            <td>255</td>
        </tr>
        <tr>
            <td>signed char</td>
            <td>1</td>
            <td>8</td>
            <td>-128</td>
            <td>127</td>
        </tr>
        <tr>
            <td>wchar_t</td>
            <td>4</td>
            <td>8</td>
            <td>-2147483648</td>
            <td>2147483647</td>
        </tr>
        <tr>
            <td>char8_t</td>
            <td>1</td>
            <td>8</td>
            <td>NaN</td>
            <td>NaN</td>
        </tr>
        <tr>
            <td>char16_t</td>
            <td>2</td>
            <td>8</td>
            <td>NaN</td>
            <td>NaN</td>
        </tr>
        <tr>
            <td>char32_t</td>
            <td>4</td>
            <td>8</td>
            <td>NaN</td>
            <td>NaN</td>
        </tr>
        <tr>
            <td>int</td>
            <td>4</td>
            <td>8</td>
            <td>-2147483648</td>
            <td>2147483647</td>
        </tr>
        <tr>
            <td>signed</td>
            <td>4</td>
            <td>8</td>
            <td>-2147483648</td>
            <td>2147483647</td>
        </tr>
        <tr>
            <td>unsigned</td>
            <td>4</td>
            <td>8</td>
            <td>0</td>
            <td>4294967295</td>
        </tr>
        <tr>
            <td>short int</td>
            <td>2</td>
            <td>8</td>
            <td>-32768</td>
            <td>32767</td>
        </tr>
        <tr>
            <td>short</td>
            <td>2</td>
            <td>8</td>
            <td>-32768</td>
            <td>32767</td>
        </tr>
        <tr>
            <td>long int</td>
            <td>8</td>
            <td>8</td>
            <td>-9223372036854775808</td>
            <td>9223372036854775807</td>
        </tr>
        <tr>
            <td>long</td>
            <td>8</td>
            <td>8</td>
            <td>-9223372036854775808</td>
            <td>9223372036854775807</td>
        </tr>
        <tr>
            <td>long long int</td>
            <td>8</td>
            <td>8</td>
            <td>-9223372036854775808</td>
            <td>9223372036854775807</td>
        </tr>
        <tr>
            <td>signed short int</td>
            <td>2</td>
            <td>8</td>
            <td>-32768</td>
            <td>32767</td>
        </tr>
        <tr>
            <td>signed int</td>
            <td>4</td>
            <td>8</td>
            <td>-2147483648</td>
            <td>2147483647</td>
        </tr>
        <tr>
            <td>signed long int</td>
            <td>8</td>
            <td>8</td>
            <td>-9223372036854775808</td>
            <td>9223372036854775807</td>
        </tr>
        <tr>
            <td>signed long long int</td>
            <td>8</td>
            <td>8</td>
            <td>-9223372036854775808</td>
            <td>9223372036854775807</td>
        </tr>
        <tr>
            <td>unsigned short int</td>
            <td>2</td>
            <td>8</td>
            <td>0</td>
            <td>65535</td>
        </tr>
        <tr>
            <td>unsigned int</td>
            <td>4</td>
            <td>8</td>
            <td>0</td>
            <td>4294967295</td>
        </tr>
        <tr>
            <td>unsigned long int</td>
            <td>8</td>
            <td>8</td>
            <td>0</td>
            <td>18446744073709551615</td>
        </tr>
        <tr>
            <td>unsigned long long int</td>
            <td>8</td>
            <td>8</td>
            <td>0</td>
            <td>18446744073709551615</td>
        </tr>
        <tr>
            <td>std::int8_t</td>
            <td>1</td>
            <td>8</td>
            <td>-128</td>
            <td>127</td>
        </tr>
        <tr>
            <td>std::uint8_t</td>
            <td>1</td>
            <td>8</td>
            <td>0</td>
            <td>255</td>
        </tr>
        <tr>
            <td>std::int16_t</td>
            <td>2</td>
            <td>8</td>
            <td>-32768</td>
            <td>32767</td>
        </tr>
        <tr>
            <td>std::uint16_t</td>
            <td>2</td>
            <td>8</td>
            <td>0</td>
            <td>65535</td>
        </tr>
        <tr>
            <td>std::int32_t</td>
            <td>4</td>
            <td>8</td>
            <td>-2147483648</td>
            <td>2147483647</td>
        </tr>
        <tr>
            <td>std::uint32_t</td>
            <td>4</td>
            <td>8</td>
            <td>0</td>
            <td>4294967295</td>
        </tr>
        <tr>
            <td>std::int64_t</td>
            <td>8</td>
            <td>8</td>
            <td>-9223372036854775808</td>
            <td>9223372036854775807</td>
        </tr>
        <tr>
            <td>std::uint64_t</td>
            <td>8</td>
            <td>8</td>
            <td>0</td>
            <td>18446744073709551615</td>
        </tr>
        <tr>
            <td>float</td>
            <td>4</td>
            <td>8</td>
            <td>1.17549e-38</td>
            <td>3.40282e+38</td>
        </tr>
        <tr>
            <td>double</td>
            <td>8</td>
            <td>8</td>
            <td>1.79769e+308</td>
            <td>2.22507e-308</td>
        </tr>
        <tr>
            <td>long double</td>
            <td>16</td>
            <td>8</td>
            <td>3.3621e-4932</td>
            <td>1.18973e+4932</td>
        </tr>
        <tr>
            <td>bool</td>
            <td>1</td>
            <td>8</td>
            <td>0</td>
            <td>1</td>
        </tr>
    </tbody>
</table>

## Explanations

### Auto Variables

**Auto** Keyword is used to automatically deduce the type of the variable from its initializer.

```cpp
auto i = 5; // i is deduced to be int
auto d = 3.14; // d is deduced to be double
auto str = " Hello " ; // str is deduced to be const char *
std :: vector < int > numbers = {1 , 2 , 3 , 4 , 5};

for ( auto it = numbers . begin () ; it != numbers . end () ; ++ it ) {
    std :: cout << * it << " " ; // it is deducted to be int
}

auto sum = []( auto a , auto b ) {
    return a + b ;
};
```

### Const

**Const** Keyword is used to declare a constant variable. Once declared, the value of the variable cannot be changed.

```cpp
const int max_size = 100;   // variable is constant
const int* ptr = &value;    // pointer can change but value is constant
int* const ptr = &value;    // pointer is constant but value change
```

### Constexpr

**Constexpr** Keyword is used to declare a constant expression. The value of the variable is evaluated at compile time.

```cpp
constexpr int array_size () {
    return 10;
}

int main () {
    int arr [ array_size () ]; // Array size evaluated at compile time
    return 0;
}
```

## Contact
Should you encounter any issues or have questions regarding the Restaurant Bot,
please reach out to Yusuf YILDIZ at [yousufy1ld1z@gmail.com](mailto:yousufy1ld1z@gmail.com).