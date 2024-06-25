# Overview
This folder includes 3 projects: Point Structure, Calculator Class Implementation, Matrix Class Implementation. 

# Requirements
    - CMake (version 3.10 or higher) 
    - C++ compiler (GCC, Clang, or MSVC)
# Installation and Setup
1. Clone the repository to your local machine.
2. Ensure a C++ compiler is installed on your system.
3. Create the build directory and navigate into it.
4. Configure the project with CMake
5. Build the project

# Project Details 
## 1. Point Structure
This project contains a C++ program for managing and manipulating 3D points (`Point3D`) and determining their regions in space. The program provides several utility functions to compute distances, compare points, and categorize points into regions.

### Features
- Define a 3D point structure `Point3D`. 
- Define an enumeration `Region` to categorize points into regions.
- Calculate the distance from a point to the origin. 
- Calculate the distance between two points. 
- Compare the distances of two points from the origin. 
- Determine the region of a point. 
- Check if two points are in the same region. 
- Print the coordinates of a point and its region.

## Structures and Enums
- **Point3D**: Represents a point in 3D space with `x`, `y`, and `z` coordinates. 
- **Region**: Enum that categorizes points into eight regions based on the sign of their coordinates, plus a `None` category for points not in any region.

## Functions 
- `inline float zero_distance(const Point3D& p1)`: Calculates the distance from point `p1` to the origin. 
- `inline float distance(const Point3D& p1, const Point3D& p2)`: Calculates the distance between points `p1` and `p2`. 
- `inline bool compare(const Point3D& p1, const Point3D& p2)`: Compares the distances of `p1` and `p2` from the origin. 
- `inline Region region(const Point3D& p)`: Determines the region of point `p`. 
- `inline bool in_same_region(const Point3D& p1, const Point3D& p2)`: Checks if points `p1` and `p2` are in the same region. 
- `inline bool in_same_subregion(const float& x1, const float& x2)`: Utility function to determine if `x1` and `x2` are in the same subregion. 
- `inline void print_point(const Point3D& p)`: Prints the coordinates of point `p`. 
- `inline void print_region(const Region& r)`: Prints the region of a point.

<div align="center">
  <img src="https://github.com/yusufy1ld1z/itusct_autonomous/assets/148093015/07377536-00e5-4a5f-a704-ac766c2aa0da" alt="Logo of ITU SCT" width="400">
</div>

## 2. Calculator
A calculator is provided that supports various arithmetic operations on these points or other numerical data types (int, float, double).

### Features
- **Calculator**: Perform basic arithmetic operations such as addition, subtraction, multiplication, division, square, exponentiation, and modulus.
- **Support for Multiple Data Types**: The calculator is implemented using templates, allowing it to work with different data types, including int, float, and double.
- **Error Handling**: The calculator includes error handling for division by zero and modulus by zero, which will throw an exception and print an error message.

### Calculator Class
The Calculator class provides the following operations:
* `T add(T a, T b)`: Returns the sum of a and b.
* `T subtract(T a, T b)`: Returns the difference between a and b.
* `T multiply(T a, T b)`: Returns the product of a and b.
* `T divide(T a, T b)`: Returns the quotient of a divided by b. Throws an exception if b is zero.
* `T square(T a)`: Returns the square of a.
* `T exp(T a, T b)`: Returns a raised to the power of b.
* `T mod(T a, T b)`: Returns the modulus of a by b. Throws an exception if b is zero.

<div align="center">
  <img src="https://github.com/yusufy1ld1z/itusct_autonomous/assets/148093015/c6ff560c-cee7-438a-a577-c81d537a9ec6" alt="Logo of ITU SCT" width="400">
</div>

## 3. Matrix

This project provides a comprehensive C++ matrix library for handling various matrix operations. The library allows for the creation, manipulation, and mathematical operations of matrices, supporting both element-wise and scalar operations.

### Features

- **Matrix Creation**: Create matrices from vectors, other matrices, or directly with dimensions.
- **Matrix Operations**: Addition, subtraction, negation, and multiplication (element-wise and matrix).
- **Utility Functions**: Determinant, inverse, trace, normalization, and magnitude.
- **Special Matrices**: Identity, zero, and ones matrices.
- **Overloaded Operators**: For easy and intuitive matrix manipulation.

#### Class `Matrix<T>`

- **Constructors**:
  - `Matrix(int rows, int cols, T value = T())`: Initializes a matrix with given rows and columns, optionally filled with a specific value.
  - `Matrix(const matrix<T>& mat)`: Initializes a matrix from a 2D vector.
  - `Matrix(const std::vector<T>& vec)`: Initializes a matrix from a 1D vector.

- **Methods**:
  - `Matrix<T> operator+(const Matrix<T>& other) const`: Adds two matrices.
  - `Matrix<T> operator-(const Matrix<T>& other) const`: Subtracts one matrix from another.
  - `Matrix<T> operator-() const`: Negates a matrix.
  - `Matrix<T> operator*(const Matrix<T>& other) const`: Multiplies two matrices element-wise.
  - `static Matrix<T> dot(const Matrix<T>& mat1, const Matrix<T>& mat2)`: Multiplies two matrices (dot product).
  - `Matrix<T> operator+(T scalar) const`: Adds a scalar to a matrix.
  - `Matrix<T> operator-(T scalar) const`: Subtracts a scalar from a matrix.
  - `Matrix<T> operator*(T scalar) const`: Multiplies a matrix by a scalar.
  - `Matrix<T> operator/(T scalar) const`: Divides a matrix by a scalar.
  - `double determinant() const`: Computes the determinant of the matrix.
  - `Matrix<T> inverse() const`: Computes the inverse of the matrix.
  - `double trace() const`: Computes the trace of the matrix.
  - `float magnitude(const Matrix<T>& mat) const`: Computes the magnitude of the matrix.
  - `Matrix<T> normalize(const Matrix<T>& mat) const`: Normalizes the matrix.
  - `static Matrix<T> zeroes(int rows, int cols)`: Creates a matrix filled with zeroes.
  - `static Matrix<T> ones(int rows, int cols)`: Creates a matrix filled with ones.
  - `static Matrix<T> identity(int rows)`: Creates an identity matrix.
  - Getters for the matrix, rows, and columns.
  - `void print_matrix(const Matrix<T>& mat) const`: Prints the matrix.

- **Overloaded Operators**:
  - `std::ostream& operator<<(std::ostream& out, const Matrix<T>& mat)`: Outputs the matrix to a stream.

<div align="center">
  <img src="https://github.com/yusufy1ld1z/itusct_autonomous/assets/148093015/d120153b-6e42-4a48-b2a6-fc04c5497897" alt="Logo of ITU SCT" width="400">
</div>

# Contact
Should you encounter any issues or have questions regarding the C++ Variables project,
please reach out to Yusuf YILDIZ at [yousufy1ld1z@gmail.com](mailto:yousufy1ld1z@gmail.com).
