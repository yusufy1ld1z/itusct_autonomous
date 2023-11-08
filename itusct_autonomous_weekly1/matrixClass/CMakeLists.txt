project(matrixClass)

set(CMAKE_CXX_STANDARD 17)

add_executable(matrixTest test.cpp)
target_include_directories(matrixTest PRIVATE include)