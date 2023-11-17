#include<iostream>
#include<cstdint>
#include<limits>
#include<iomanip>
#include<string>
// #include<cwchar>

void print_size();
void print_limits();

// C++ Variables
int main() {
    
    // C++ char types
    char m_char = 'a';
    unsigned char m_uchar = 'b';
    signed char m_schar = 'c';
    wchar_t m_wchar = 'd';
    // std::wcout << "wchar_t: " << m_wchar << std::endl; for printing, wcout must be used
    char8_t m_char8 = 'e';
    char16_t m_char16 = 'e';
    char32_t m_char32 = 'f';

    // C++ integer types
    int m_int = 1;
    signed m_signed = 1;
    unsigned m_unsigned = 1;
    short int m_short = 2;
    short m_short2 = 2;
    long int m_long = 3;
    long m_long2 = 3;
    long long int m_longlong = 4;
    signed short int m_sshort = 1;
    signed int m_sint = 2;
    signed long int m_slong = 3;    
    signed long long int m_slonglong = 4;
    unsigned short int m_ushort = 5;
    unsigned int m_uint = 6;
    unsigned long int m_ulong = 7;
    unsigned long long int m_ulonglong = 8;
    
    // these types are built in std
    std::int8_t m_int8 = 9;                 
    std::uint8_t m_uint8 = 10;
    std::int16_t m_int16 = 9;
    std::uint16_t m_uint16 = 10;
    std::int32_t m_int32 = 11;
    std::uint32_t m_uint32 = 12;
    std::int64_t m_int64 = 13;
    std::uint64_t m_uint64 = 14;

    // C++ floating point types
    float m_float = 1.0;
    double m_double = 2.0;
    long double m_longdouble = 3.0; 
    
    // these types are not built in std because they are newly introduced in c++23
    
    // std::float16_t m_float16 = 1.0;
    // std::float32_t m_float32 = 2.0;
    // std::float64_t m_float64 = 3.0;
    // std::float128_t m_float128 = 4.0;
    // std::bfloat16_t m_bfloat16 = 5.0;
    
    // C++ boolean types
    bool m_bool = true;

    // Pointers to the declared types
    char* ptr_char = &m_char;
    unsigned char* ptr_uchar = &m_uchar;
    signed char* ptr_schar = &m_schar;
    wchar_t* ptr_wchar = &m_wchar;
    char8_t* ptr_char8 = &m_char8;
    char16_t* ptr_char16 = &m_char16;
    char32_t* ptr_char32 = &m_char32;

    int* ptr_int = &m_int;
    signed* ptr_signed = &m_signed;
    unsigned* ptr_unsigned = &m_unsigned;
    short int* ptr_short = &m_short;
    short* ptr_short2 = &m_short2;
    long int* ptr_long = &m_long;
    long* ptr_long2 = &m_long2;
    long long int* ptr_longlong = &m_longlong;
    signed short int* ptr_sshort = &m_sshort;
    signed int* ptr_sint = &m_sint;
    signed long int* ptr_slong = &m_slong;
    signed long long int* ptr_slonglong = &m_slonglong;
    unsigned short int* ptr_ushort = &m_ushort;
    unsigned int* ptr_uint = &m_uint;
    unsigned long int* ptr_ulong = &m_ulong;
    unsigned long long int* ptr_ulonglong = &m_ulonglong;
    std::int8_t* ptr_int8 = &m_int8;
    std::uint8_t* ptr_uint8 = &m_uint8;
    std::int16_t* ptr_int16 = &m_int16;
    std::uint16_t* ptr_uint16 = &m_uint16;
    std::int32_t* ptr_int32 = &m_int32;
    std::uint32_t* ptr_uint32 = &m_uint32;
    std::int64_t* ptr_int64 = &m_int64;
    std::uint64_t* ptr_uint64 = &m_uint64;

    float* ptr_float = &m_float;
    double* ptr_double = &m_double;
    long double* ptr_longdouble = &m_longdouble;
    
    // these types are not built in std because they are newly introduced in c++23
    
    // std::float16_t* ptr_float16 = &m_float16;
    // std::float32_t* ptr_float32 = &m_float32;
    // std::float64_t* ptr_float64 = &m_float64;
    // std::float128_t* ptr_float128 = &m_float128;
    // std::bfloat16_t* ptr_bfloat16 = &m_bfloat16;

    bool* ptr_bool = &m_bool;

    // std::sizeof()
    print_size();
    // std::numeric_limits<>
    print_limits();
    return 0;
}

void print_size(){
    int w1 = 25;
    int w2 = 20;
    int total_width = w1 + w2;
    std::string line(total_width, '-');
    std::cout << line << std::endl;
    std::cout << std::left << std::setw(w1) << "Type" << std::setw(w2) << "Size" << std::endl;
    std::cout << line << std::endl;
    std::cout << std::setw(w1) << "char" << std::setw(w2) << sizeof(char) << std::endl;
    std::cout << std::setw(w1) << "unsigned char" << std::setw(w2) << sizeof(unsigned char) << std::endl;
    std::cout << std::setw(w1) << "signed char" << std::setw(w2) << sizeof(signed char) << std::endl;
    std::cout << std::setw(w1) << "wchar_t" << std::setw(w2) << sizeof(wchar_t) << std::endl;
    std::cout << std::setw(w1) << "char8_t" << std::setw(w2) << sizeof(char8_t) << std::endl;
    std::cout << std::setw(w1) << "char16_t" << std::setw(w2) << sizeof(char16_t) << std::endl;
    std::cout << std::setw(w1) << "char32_t" << std::setw(w2) << sizeof(char32_t) << std::endl;

    // C++ integer types
    std::cout << std::setw(w1) << "int" << std::setw(w2) << sizeof(int) << std::endl;
    std::cout << std::setw(w1) << "signed" << std::setw(w2) << sizeof(signed) << std::endl;
    std::cout << std::setw(w1) << "unsigned" << std::setw(w2) << sizeof(unsigned) << std::endl;
    std::cout << std::setw(w1) << "short int" << std::setw(w2) << sizeof(short int) << std::endl;
    std::cout << std::setw(w1) << "short" << std::setw(w2) << sizeof(short) << std::endl;
    std::cout << std::setw(w1) << "long int" << std::setw(w2) << sizeof(long int) << std::endl;
    std::cout << std::setw(w1) << "long" << std::setw(w2) << sizeof(long) << std::endl;
    std::cout << std::setw(w1) << "long long int" << std::setw(w2) << sizeof(long long int) << std::endl;
    std::cout << std::setw(w1) << "signed short int" << std::setw(w2) << sizeof(signed short int) << std::endl;
    std::cout << std::setw(w1) << "signed int" << std::setw(w2) << sizeof(signed int) << std::endl;
    std::cout << std::setw(w1) << "signed long int" << std::setw(w2) << sizeof(signed long int) << std::endl;
    std::cout << std::setw(w1) << "signed long long int" << std::setw(w2) << sizeof(signed long long int) << std::endl;
    std::cout << std::setw(w1) << "unsigned short int" << std::setw(w2) << sizeof(unsigned short int) << std::endl;
    std::cout << std::setw(w1) << "unsigned int" << std::setw(w2) << sizeof(unsigned int) << std::endl;
    std::cout << std::setw(w1) << "unsigned long int" << std::setw(w2) << sizeof(unsigned long int) << std::endl;
    std::cout << std::setw(w1) << "unsigned long long int" << std::setw(w2) << sizeof(unsigned long long int) << std::endl;
    std::cout << std::setw(w1) << "std::int8_t" << std::setw(w2) << sizeof(std::int8_t) << std::endl;
    std::cout << std::setw(w1) << "std::uint8_t" << std::setw(w2) << sizeof(std::uint8_t) << std::endl;
    std::cout << std::setw(w1) << "std::int16_t" << std::setw(w2) << sizeof(std::int16_t) << std::endl;
    std::cout << std::setw(w1) << "std::uint16_t" << std::setw(w2) << sizeof(std::uint16_t) << std::endl;
    std::cout << std::setw(w1) << "std::int32_t" << std::setw(w2) << sizeof(std::int32_t) << std::endl;
    std::cout << std::setw(w1) << "std::uint32_t" << std::setw(w2) << sizeof(std::uint32_t) << std::endl;
    std::cout << std::setw(w1) << "std::int64_t" << std::setw(w2) << sizeof(std::int64_t) << std::endl;
    std::cout << std::setw(w1) << "std::uint64_t" << std::setw(w2) << sizeof(std::uint64_t) << std::endl;

    // C++ floating point types
    std::cout << std::setw(w1) << "float" << std::setw(w2) << sizeof(float) << std::endl;
    std::cout << std::setw(w1) << "double" << std::setw(w2) << sizeof(double) << std::endl;
    std::cout << std::setw(w1) << "long double" << std::setw(w2) << sizeof(long double) << std::endl;
    
    // these types are not built in std because they are newly introduced in c++23
    
    // std::cout << "sizeof(std::float16_t*): " << sizeof(float16_t*) << std::endl;
    // std::cout << "sizeof(std::float32_t*): " << sizeof(float32_t*) << std::endl;
    // std::cout << "sizeof(std::float64_t*): " << sizeof(float64_t*) << std::endl;
    // std::cout << "sizeof(std::float128_t*): " << sizeof(float128_t*) << std::endl;
    // std::cout << "sizeof(std::bfloat16_t*): " << sizeof(bfloat16_t*) << std::endl;

    // C++ boolean types
    std::cout << std::setw(w1) << "bool" << std::setw(w2) << sizeof(bool) << std::endl;

    // Pointers
    std::cout << line << std::endl;
    std::cout << std::setw(w1) << "char*" << std::setw(w2) << sizeof(char*) << std::endl;
    std::cout << std::setw(w1) << "unsigned char*" << std::setw(w2) << sizeof(unsigned char*) << std::endl;
    std::cout << std::setw(w1) << "signed char*" << std::setw(w2) << sizeof(signed char*) << std::endl;
    std::cout << std::setw(w1) << "wchar_t*" << std::setw(w2) << sizeof(wchar_t*) << std::endl;
    std::cout << std::setw(w1) << "char8_t*" << std::setw(w2) << sizeof(char8_t*) << std::endl;
    std::cout << std::setw(w1) << "char16_t*" << std::setw(w2) << sizeof(char16_t*) << std::endl;
    std::cout << std::setw(w1) << "char32_t*" << std::setw(w2) << sizeof(char32_t*) << std::endl;

    std::cout << std::setw(w1) << "int*" << std::setw(w2) << sizeof(int*) << std::endl;
    std::cout << std::setw(w1) << "signed*" << std::setw(w2) << sizeof(signed*) << std::endl;
    std::cout << std::setw(w1) << "unsigned*" << std::setw(w2) << sizeof(unsigned*) << std::endl;
    std::cout << std::setw(w1) << "short int*" << std::setw(w2) << sizeof(short int*) << std::endl;
    std::cout << std::setw(w1) << "short*" << std::setw(w2) << sizeof(short*) << std::endl;
    std::cout << std::setw(w1) << "long int*" << std::setw(w2) << sizeof(long int*) << std::endl;
    std::cout << std::setw(w1) << "long*" << std::setw(w2) << sizeof(long*) << std::endl;
    std::cout << std::setw(w1) << "long long int*" << std::setw(w2) << sizeof(long long int*) << std::endl;
    std::cout << std::setw(w1) << "signed short int*" << std::setw(w2) << sizeof(signed short int*) << std::endl;
    std::cout << std::setw(w1) << "signed int*" << std::setw(w2) << sizeof(signed int*) << std::endl;
    std::cout << std::setw(w1) << "signed long int*" << std::setw(w2) << sizeof(signed long int*) << std::endl;
    std::cout << std::setw(w1) << "signed long long int*" << std::setw(w2) << sizeof(signed long long int*) << std::endl;
    std::cout << std::setw(w1) << "unsigned short int*" << std::setw(w2) << sizeof(unsigned short int*) << std::endl;
    std::cout << std::setw(w1) << "unsigned int*" << std::setw(w2) << sizeof(unsigned int*) << std::endl;
    std::cout << std::setw(w1) << "unsigned long int*" << std::setw(w2) << sizeof(unsigned long int*) << std::endl;
    std::cout << std::setw(w1) << "unsigned long long int*" << std::setw(w2) << sizeof(unsigned long long int*) << std::endl;
    std::cout << std::setw(w1) << "std::int8_t*" << std::setw(w2) << sizeof(std::int8_t*) << std::endl;
    std::cout << std::setw(w1) << "std::uint8_t*" << std::setw(w2) << sizeof(std::uint8_t*) << std::endl;
    std::cout << std::setw(w1) << "std::int16_t*" << std::setw(w2) << sizeof(std::int16_t*) << std::endl;
    std::cout << std::setw(w1) << "std::uint16_t*" << std::setw(w2) << sizeof(std::uint16_t*) << std::endl;
    std::cout << std::setw(w1) << "std::int32_t*" << std::setw(w2) << sizeof(std::int32_t*) << std::endl;
    std::cout << std::setw(w1) << "std::uint32_t*" << std::setw(w2) << sizeof(std::uint32_t*) << std::endl;
    std::cout << std::setw(w1) << "std::int64_t*" << std::setw(w2) << sizeof(std::int64_t*) << std::endl;
    std::cout << std::setw(w1) << "std::uint64_t*" << std::setw(w2) << sizeof(std::uint64_t*) << std::endl;

    // C++ floating point types
    std::cout << std::setw(w1) << "float*" << std::setw(w2) << sizeof(float*) << std::endl;
    std::cout << std::setw(w1) << "double*" << std::setw(w2) << sizeof(double*) << std::endl;
    std::cout << std::setw(w1) << "long double*" << std::setw(w2) << sizeof(long double*) << std::endl;
    
    // these types are not built in std because they are newly introduced in c++23
    
    // std::cout << "sizeof(std::float16_t*): " << sizeof(float16_t*) << std::endl;
    // std::cout << "sizeof(std::float32_t*): " << sizeof(float32_t*) << std::endl;
    // std::cout << "sizeof(std::float64_t*): " << sizeof(float64_t*) << std::endl;
    // std::cout << "sizeof(std::float128_t*): " << sizeof(float128_t*) << std::endl;
    // std::cout << "sizeof(std::bfloat16_t*): " << sizeof(bfloat16_t*) << std::endl;

    // C++ boolean types
    std::cout << std::setw(w1) << "bool*" << std::setw(w2) << sizeof(bool*) << std::endl;
}

void print_limits(){

/*
    std::numeric_limits function is defined for the following types:

    template<class T> class numeric_limits;
 
    template<class T> class numeric_limits<const T>;
    template<class T> class numeric_limits<volatile T>;
    template<class T> class numeric_limits<const volatile T>;
 
    template<> class numeric_limits<bool>;
 
    template<> class numeric_limits<char>;
    template<> class numeric_limits<signed char>;
    template<> class numeric_limits<unsigned char>;
    template<> class numeric_limits<char8_t>;
    template<> class numeric_limits<char16_t>;
    template<> class numeric_limits<char32_t>;
    template<> class numeric_limits<wchar_t>;
 
    template<> class numeric_limits<short>;
    template<> class numeric_limits<int>;
    template<> class numeric_limits<long>;
    template<> class numeric_limits<long long>;
    template<> class numeric_limits<unsigned short>;
    template<> class numeric_limits<unsigned int>;
    template<> class numeric_limits<unsigned long>;
    template<> class numeric_limits<unsigned long long>;
 
    template<> class numeric_limits<float>;
    template<> class numeric_limits<double>;
    template<> class numeric_limits<long double>;
*/
    // C++ char types
    int w1 = 25;
    int w2 = 25;
    int w3 = 20;
    int total_width = w1 + w2 + w3;
    std::string line(total_width, '-');
    std::cout << line << std::endl;
    std::cout << std::left << std::setw(w1) << "Type" << std::setw(w2) << "Min" << std::setw(w3) << "Max" << std::endl;
    std::cout << line << std::endl;
    std::cout << std::setw(w1) << "char" << std::setw(w2) << static_cast<int>(std::numeric_limits<char>::min()) << std::setw(w3) << static_cast<int>(std::numeric_limits<char>::max()) << std::endl;
    std::cout << std::setw(w1) << "unsigned char" << std::setw(w2) << static_cast<int>(std::numeric_limits<unsigned char>::min()) << std::setw(w3) << static_cast<int>(std::numeric_limits<unsigned char>::max()) << std::endl;
    std::cout << std::setw(w1) << "signed char" << std::setw(w2) << static_cast<int>(std::numeric_limits<signed char>::min()) << std::setw(w3) << static_cast<int>(std::numeric_limits<signed char>::max()) << std::endl;
    /*The operator<< for char32_t is deleted in C++ standard to prevent it from being mistakenly printed as a numeric value. 
    This is because char32_t is used to represent Unicode characters, and the intention is to prevent it from being output as an integer.*/

    // std::cout << "wchar_t min: " << std::numeric_limits<wchar_t>::min() << ", max: " << std::numeric_limits<wchar_t>::max() << std::endl;
    // std::cout << "char8_t min: " << std::numeric_limits<char8_t>::min() << ", max: " << std::numeric_limits<char8_t>::max() << std::endl;
    // std::cout << "char16_t min: " << std::numeric_limits<char16_t>::min() << ", max: " << std::numeric_limits<char16_t>::max() << std::endl;
    // std::cout << "char32_t min: " << std::numeric_limits<char32_t>::min() << ", max: " << std::numeric_limits<char32_t>::max() << std::endl;

    // C++ integer types
    std::cout << std::setw(w1) << "int" << std::setw(w2) << std::numeric_limits<int>::min() << std::setw(w3) << std::numeric_limits<int>::max() << std::endl;
    std::cout << std::setw(w1) << "signed" << std::setw(w2) << std::numeric_limits<signed>::min() << std::setw(w3) << std::numeric_limits<signed>::max() << std::endl;
    std::cout << std::setw(w1) << "unsigned" << std::setw(w2) << std::numeric_limits<unsigned>::min() << std::setw(w3) << std::numeric_limits<unsigned>::max() << std::endl;
    std::cout << std::setw(w1) << "short int" << std::setw(w2) << std::numeric_limits<short int>::min() << std::setw(w3) << std::numeric_limits<short int>::max() << std::endl;
    std::cout << std::setw(w1) << "short" << std::setw(w2) << std::numeric_limits<short>::min() << std::setw(w3) << std::numeric_limits<short>::max() << std::endl;
    std::cout << std::setw(w1) << "long int" << std::setw(w2) << std::numeric_limits<long int>::min() << std::setw(w3) << std::numeric_limits<long int>::max() << std::endl;
    std::cout << std::setw(w1) << "long" << std::setw(w2) << std::numeric_limits<long>::min() << std::setw(w3) << std::numeric_limits<long>::max() << std::endl;
    std::cout << std::setw(w1) << "long long int" << std::setw(w2) << std::numeric_limits<long long int>::min() << std::setw(w3) << std::numeric_limits<long long int>::max() << std::endl;
    std::cout << std::setw(w1) << "signed short int" << std::setw(w2) << std::numeric_limits<signed short int>::min() << std::setw(w3) << std::numeric_limits<signed short int>::max() << std::endl;
    std::cout << std::setw(w1) << "signed int" << std::setw(w2) << std::numeric_limits<signed int>::min() << std::setw(w3) << std::numeric_limits<signed int>::max() << std::endl;
    std::cout << std::setw(w1) << "signed long int" << std::setw(w2) << std::numeric_limits<signed long int>::min() << std::setw(w3) << std::numeric_limits<signed long int>::max() << std::endl;
    std::cout << std::setw(w1) << "signed long long int" << std::setw(w2) << std::numeric_limits<signed long long int>::min() << std::setw(w3) << std::numeric_limits<signed long long int>::max() << std::endl;
    std::cout << std::setw(w1) << "unsigned short int" << std::setw(w2) << std::numeric_limits<unsigned short int>::min() << std::setw(w3) << std::numeric_limits<unsigned short int>::max() << std::endl;
    std::cout << std::setw(w1) << "unsigned int" << std::setw(w2) << std::numeric_limits<unsigned int>::min() << std::setw(w3) << std::numeric_limits<unsigned int>::max() << std::endl;
    std::cout << std::setw(w1) << "unsigned long int" << std::setw(w2) << std::numeric_limits<unsigned long int>::min() << std::setw(w3) << std::numeric_limits<unsigned long int>::max() << std::endl;
    std::cout << std::setw(w1) << "unsigned long long int" << std::setw(w2) << std::numeric_limits<unsigned long long int>::min() << std::setw(w3) << std::numeric_limits<unsigned long long int>::max() << std::endl;
    std::cout << std::setw(w1) << "std::int8_t" << std::setw(w2) << static_cast<int>(std::numeric_limits<std::int8_t>::min()) << std::setw(w3) << static_cast<int>(std::numeric_limits<std::int8_t>::max()) << std::endl;
    std::cout << std::setw(w1) << "std::uint8_t" << std::setw(w2) << static_cast<int>(std::numeric_limits<std::uint8_t>::min()) << std::setw(w3) << static_cast<int>(std::numeric_limits<std::uint8_t>::max()) << std::endl;
    std::cout << std::setw(w1) << "std::int16_t" << std::setw(w2) << std::numeric_limits<std::int16_t>::min() << std::setw(w3) << std::numeric_limits<std::int16_t>::max() << std::endl;
    std::cout << std::setw(w1) << "std::uint16_t" << std::setw(w2) << std::numeric_limits<std::uint16_t>::min() << std::setw(w3) << std::numeric_limits<std::uint16_t>::max() << std::endl;
    std::cout << std::setw(w1) << "std::int32_t" << std::setw(w2) << std::numeric_limits<std::int32_t>::min() << std::setw(w3) << std::numeric_limits<std::int32_t>::max() << std::endl;
    std::cout << std::setw(w1) << "std::uint32_t" << std::setw(w2) << std::numeric_limits<std::uint32_t>::min() << std::setw(w3) << std::numeric_limits<std::uint32_t>::max() << std::endl;
    std::cout << std::setw(w1) << "std::int64_t" << std::setw(w2) << std::numeric_limits<std::int64_t>::min() << std::setw(w3) << std::numeric_limits<std::int64_t>::max() << std::endl;
    std::cout << std::setw(w1) << "std::uint64_t" << std::setw(w2) << std::numeric_limits<std::uint64_t>::min() << std::setw(w3) << std::numeric_limits<std::uint64_t>::max() << std::endl;

    // C++ floating point types
    std::cout << std::setw(w1) << "float" << std::setw(w2) << std::numeric_limits<float>::min() << std::setw(w3) << std::numeric_limits<float>::max() << std::endl;
    std::cout << std::setw(w1) << "double" << std::setw(w2) << std::numeric_limits<double>::min() << std::setw(w3) << std::numeric_limits<double>::max() << std::endl;
    std::cout << std::setw(w1) << "long double" << std::setw(w2) << std::numeric_limits<long double>::min() << std::setw(w3) << std::numeric_limits<long double>::max() << std::endl;
    
    // these types are not built in std because they are newly introduced in c++23

    // cout << "numeric_limits<std::float16_t>::min(): " << std::numeric_limits<float16_t>::min() << ", max: " << std::numeric_limits<std::float16_t>::max() << endl;
    // cout << "numeric_limits<std::float32_t>::min(): " << std::numeric_limits<float32_t>::min() << ", max: " << std::numeric_limits<std::float32_t>::max() << endl;
    // cout << "numeric_limits<std::float64_t>::min(): " << std::numeric_limits<float64_t>::min() << ", max: " << std::numeric_limits<std::float64_t>::max() << endl;
    // cout << "numeric_limits<std::float128_t>::min(): " << std::numeric_limits<float128_t>::min() << ", max: " << std::numeric_limits<std::float128_t>::max() << endl;
    // cout << "numeric_limits<std::bfloat16_t>::min(): " << std::numeric_limits<bfloat16_t>::min() << ", max: " << std::numeric_limits<std::bfloat16_t>::max() << endl;

    // C++ boolean types
    std::cout << std::setw(w1) << "bool" << std::setw(w2) << std::numeric_limits<bool>::min() << std::setw(w3) << std::numeric_limits<bool>::max() << std::endl;
}