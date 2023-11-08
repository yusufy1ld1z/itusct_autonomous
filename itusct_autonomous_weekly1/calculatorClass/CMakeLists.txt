project(calculatorClass)

set(CMAKE_CXX_STANDARD 17)

add_executable(calculatorTest test.cpp)
target_include_directories(calculatorTest PRIVATE include)