# C++ Pointers and Smart Pointers Project

## Overview

This project is a comprehensive exploration of pointers and smart pointers in C++. It includes the creation of a custom SmartPointer class and demonstrates the usage of various standard smart pointers (auto_ptr, unique_ptr, shared_ptr, and weak_ptr). The project aims to illustrate the concepts of memory management, ownership models, and the prevention of common pointer-related issues such as memory leaks, dangling pointers, and cyclic dependencies.

## Table of Contents
- [Custom SmartPointer](#custom-smartpointer)
- [Standard Smart Pointers](#standard-smart-pointers)
  - [auto_ptr](#auto_ptr)
  - [unique_ptr](#unique_ptr)
  - [shared_ptr](#shared_ptr)
  - [weak_ptr](#weak_ptr)
- [Garbage Collection Mechanism](#garbage-collection-mechanism)
- [Cons of Raw Pointers](#cons-of-raw-pointers)
- [Usage](#usage)
- [Ownership Models](#ownership-models)
- [Conclusion](#conclusion)
- [Contact](#contact)



## Custom SmartPointer

The SmartPointer class is a custom implementation of a smart pointer. It includes:
- A constructor that initializes the pointer.
- A destructor that deletes the pointer and sets it to nullptr.
- Overloaded * and -> operators to access the object pointed to.

### Code Example
```cpp
template <typename T>
class SmartPointer {
private:
    T* ptr;

public:
    SmartPointer(T* p) : ptr(p) {
        std::cout << "Custom SmartPointer created with address: " << ptr << std::endl;
    }

    ~SmartPointer() {
        std::cout << "Custom SmartPointer destroyed. Address: " << ptr << std::endl;
        delete ptr;
        ptr = nullptr;
    }

    T& operator*() {
        return *ptr;
    }

    T* operator->() {
        return ptr;
    }
};
```

### Explanation

1. *Constructor*:
    cpp
    SmartPointer(T* p) : ptr(p) {
        std::cout << "Custom SmartPointer created with address: " << ptr << std::endl;
    }
    
    The constructor initializes the raw pointer ptr and outputs its address, indicating that a SmartPointer has been created and is managing the given memory address.

2. *Destructor*:
    cpp
    ~SmartPointer() {
        std::cout << "Custom SmartPointer destroyed. Address: " << ptr << std::endl;
        delete ptr;
        ptr = nullptr;
    }
    
    The destructor is called when the SmartPointer object goes out of scope. It deletes the raw pointer to free the allocated memory and sets the pointer to nullptr to avoid dangling pointers. This ensures proper cleanup and prevents memory leaks.

3. **Overloaded * Operator**:
    cpp
    T& operator*() {
        return *ptr;
    }
    
    This operator allows you to dereference the SmartPointer object, accessing the value of the object it points to. It provides the same functionality as dereferencing a raw pointer.

4. *Overloaded -> Operator*:
    cpp
    T* operator->() {
        return ptr;
    }
    
    This operator allows you to access the members of the object pointed to by the SmartPointer. It provides the same functionality as using the arrow operator with a raw pointer, enabling member access of the pointed-to object.


## Standard Smart Pointers

### auto_ptr
auto_ptr is deprecated in C++11 due to its unsafe ownership transfer semantics. It is recommended to use unique_ptr instead for better safety and ownership management.

### unique_ptr
A unique_ptr is a smart pointer that owns and manages another object through a pointer and disposes of that object when the unique_ptr goes out of scope. It ensures that there is only one unique_ptr instance that owns a particular object at a time. Demonstrates usage of get() and move() functions.

### shared_ptr
A shared_ptr is a smart pointer that retains shared ownership of an object through a pointer. Multiple shared_ptr objects may own the same object. Demonstrates usage of get(), move(), use_count(), and reset() functions.

### weak_ptr
A weak_ptr is a smart pointer that holds a non-owning ("weak") reference to an object that is managed by shared_ptr. It is used to prevent cyclic dependencies that can lead to memory leaks. Demonstrates usage of lock() and expired() functions.



## Garbage Collection Mechanism

Smart pointers in C++ provide a form of automatic garbage collection by ensuring that objects are deleted when they are no longer needed. This helps in managing the lifetime of dynamically allocated objects and prevents memory leaks.


## Cons of Raw Pointers

Raw pointers can lead to several issues:

- *Memory leaks*: Forgetting to delete allocated memory.
- *Dangling pointers*: Accessing memory that has been deleted.
- *Wild pointers*: Uninitialized pointers that point to arbitrary memory.
- *Data inconsistency*: Multiple pointers modifying the same data without proper synchronization.
- *Buffer overflow*: Accessing memory outside the bounds of an allocated array.



## Usage

The main program demonstrates various functionalities of custom and standard library smart pointers. Below is a brief usage example for each type of smart pointer:

### Custom SmartPointer

The SmartPointer class manages a raw pointer and ensures proper memory management.

1. *Creating and using a SmartPointer*:
    ```cpp
    SmartPointer<int> customPtr(new int(42));
    std::cout << "Value through customPtr: " << *customPtr << std::endl;
    ```

    This creates a SmartPointer managing an integer with the value 42 and prints the value.

### std::unique_ptr

std::unique_ptr is a smart pointer that owns a dynamically allocated object and ensures its deletion when the unique_ptr goes out of scope.

1. *Creating and using a unique_ptr*:
    ```cpp
    std::unique_ptr<int> uniquePtr = std::make_unique<int>(36);
    std::cout << "Value through uniquePtr: " << *uniquePtr << std::endl;
    ```

    This creates a unique_ptr managing an integer with the value 36 and prints the value.

### std::shared_ptr

std::shared_ptr is a smart pointer that allows multiple pointers to share ownership of a dynamically allocated object.

1. *Creating and using shared_ptr instances*:
    ```cpp
    std::shared_ptr<int> sharedPtr1 = std::make_shared<int>(50);
    std::cout << "Value through sharedPtr1: " << *sharedPtr1 << " (use_count: " << sharedPtr1.use_count() << ")" << std::endl;
    ```

    This creates a shared_ptr managing an integer with the value 50, prints the value, and shows the number of shared owners.

### std::weak_ptr

std::weak_ptr is a non-owning smart pointer that holds a weak reference to an object managed by std::shared_ptr.

1. *Creating and using a weak_ptr*:
    ```cpp
    int main() {
        std::shared_ptr<int> ptr1; // Default constructed, empty
        if (ptr1 == nullptr) {
            std::cout << "ptr1 is empty (nullptr)." << std::endl;
        }

        std::shared_ptr<int> ptr2 = std::make_shared<int>(10);
        ptr2.reset(); // Reset to empty state
        if (ptr2 == nullptr) {
            std::cout << "ptr2 is now empty (nullptr) after reset." << std::endl;
        }

        std::shared_ptr<int> ptr3 = std::make_shared<int>(20);
        std::weak_ptr<int> weakPtr = ptr3;
        ptr3.reset(); // Releases managed object

        std::shared_ptr<int> ptr4 = weakPtr.lock(); // Attempt to lock the weak_ptr
        if (ptr4 == nullptr) {
            std::cout << "ptr4 is empty (nullptr) because weakPtr expired." << std::endl;
        } else {
            std::cout << "value through weakPtr.lock(): " << *ptr4 << std::endl;
        }

        return 0;
    }
    ```
    
    This creates a weak_ptr referring to an integer managed by ptr3, locks it to create a shared_ptr, and prints the value if successful.

Note that when creating a smart pointer it is considered as the best practice to use `make_* (make_unique, make_shared, etc.)` constructors instead of direct initialization like `std::unique_ptr<int> uniquePtr(new int(36));` for several reasons:

1. Exception Safety:
- make_* functions: Ensure that memory allocation and object construction happen together, which makes them exception-safe. If an exception is thrown during the construction of the object, the memory will be correctly deallocated.

- Direct initialization: If an exception is thrown between the new allocation and the assignment to the smart pointer, there is a risk of a memory leak because the allocated memory might not be correctly deallocated.

2. Code Conciseness and Clarity:
- make_* functions: More concise and expressive, clearly showing the intent of creating a smart pointer.

- Direct initialization: More verbose and less clear, requiring explicit use of new.

3. Performance:
- std::make_shared: Can be more efficient than direct initialization because it allocates the memory for both the object and the control block in a single allocation, which can reduce memory fragmentation and improve cache performance.

- Direct initialization: Requires two separate allocations—one for the object and one for the control block—when using std::shared_ptr, which can be less efficient.

4. Best Practices:
- make_* functions: Recommended by the C++ Core Guidelines and widely considered the best practice for creating smart pointers.

- Direct initialization: Generally discouraged except in specific cases where make_* functions cannot be used (e.g., custom deleters or incomplete types).

## Ownership Models

- *unique_ptr*: Sole ownership. Only one unique_ptr can own the object at a time.
- *shared_ptr*: Shared ownership. Multiple shared_ptr instances can own the same object.
- *weak_ptr*: Non-owning reference to an object managed by shared_ptr. Used to break cyclic dependencies.

## Conclusion

This project provides a detailed exploration of C++ pointers and smart pointers, demonstrating their usage, benefits, and the mechanisms they offer to manage memory safely and efficiently. The included code examples and explanations serve as a valuable learning resource for understanding and implementing smart pointers in C++.

## Contact
Should you encounter any issues or have questions regarding the C++ Variables project, please reach out to Yusuf YILDIZ at yousufy1ld1z@gmail.com.
