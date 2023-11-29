// Author: Javad Ibrahimli

#include <iostream>
#include <memory>

// Create your own SmartPointer class
template <typename T>
class SmartPointer {
private:
    T* ptr; // Raw pointer to manage

public:
    // Constructor
    SmartPointer(T* p) : ptr(p) {
        std::cout << "Custom SmartPointer created with address: " << ptr << std::endl;
    }

    // Destructor
    ~SmartPointer() {
        std::cout << "Custom SmartPointer destroyed. Address: " << ptr << std::endl;
        delete ptr;
        ptr = nullptr;
    }

    // Overload operator* to dereference the pointer
    T& operator*() {
        return *ptr;
    }

    // Overload operator-> to access members of the object
    T* operator->() {
        return ptr;
    }
};

int main() {
    // Create a custom SmartPointer and test it
    SmartPointer<int> customPtr(new int(42));
    std::cout << "Value through customPtr: " << *customPtr << std::endl;

    // Add std::unique_ptr variable
    std::unique_ptr<int> uniquePtr(new int(36));
    std::cout << "Value through uniquePtr: " << *uniquePtr << std::endl;

    // Use get() and move() function of std::unique_ptr
    int* uniquePtrValue = uniquePtr.get();
    std::cout << "Value through get() of uniquePtr: " << *uniquePtrValue << std::endl;

    std::unique_ptr<int> uniquePtrMoved = std::move(uniquePtr);
    std::cout << "Value through uniquePtrMoved: " << *uniquePtrMoved << std::endl;

    // Explanation of auto_ptr deprecation and unique_ptr usage
    // std::auto_ptr is deprecated because it has ownership transfer semantics
    // which can lead to unexpected behavior. std::unique_ptr, introduced in C++11,
    // has improved ownership semantics and better safety features.

    // Add std::shared_ptr variable
    std::shared_ptr<int> sharedPtr1 = std::make_shared<int>(50);
    std::shared_ptr<int> sharedPtr2 = sharedPtr1; // Shared ownership

    std::cout << "Value through sharedPtr1: " << *sharedPtr1 << " (use_count: " << sharedPtr1.use_count() << ")" << std::endl;
    std::cout << "Value through sharedPtr2: " << *sharedPtr2 << " (use_count: " << sharedPtr2.use_count() << ")" << std::endl;

    // Use get(), move(), use_count(), and reset() functions of std::shared_ptr
    int* sharedPtrValue = sharedPtr1.get();
    std::cout << "Value through get() of sharedPtr1: " << *sharedPtrValue << std::endl;

    std::shared_ptr<int> sharedPtrMoved = std::move(sharedPtr1);
    std::cout << "Value through sharedPtrMoved: " << *sharedPtrMoved << " (use_count: " << sharedPtrMoved.use_count() << ")" << std::endl;

    sharedPtr2.reset(); // Reset one instance
    std::cout << "sharedPtr2 reset. sharedPtr1 use_count: " << sharedPtr1.use_count() << std::endl;

    // Add std::weak_ptr variable
    std::shared_ptr<int> sharedPtr3 = std::make_shared<int>(75);
    std::weak_ptr<int> weakPtr = sharedPtr3;

    // Use get() and move() functions of std::weak_ptr
    std::shared_ptr<int> sharedPtrFromWeak = weakPtr.lock();
    std::cout << "Value through weakPtr.lock(): " << *sharedPtrFromWeak << std::endl;

    std::weak_ptr<int> weakPtrMoved = std::move(weakPtr);
    std::shared_ptr<int> sharedPtrFromMovedWeak = weakPtrMoved.lock();
    std::cout << "Value through weakPtrMoved.lock(): " << *sharedPtrFromMovedWeak << std::endl;

    // Explanation of cyclic dependency and why weak_ptr is used
    // Weak pointers break cyclic dependencies among shared pointers, preventing memory leaks.

    return 0;
}