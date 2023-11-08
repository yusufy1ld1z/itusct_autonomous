#include<iostream>
#include<cmath>

typedef enum{
    First = 1,
    Second,
    Third,
    Fourth,
    Fifth,
    Sixth,
    Seventh,
    Eighth,
    None
} Region;

typedef struct 
{
    float x;
    float y;
    float z;
}Point3D;

inline float zero_distance(const Point3D& p1);                    // Distance from p1 to origin
inline float distance(const Point3D& p1, const Point3D& p2);      // Distance between p1 and p2
inline bool compare(const Point3D& p1, const Point3D& p2);        // Compare the distance between zero and respectively p1 and p2
inline Region region(const Point3D& p);                           // Determine the region of p
inline bool in_same_region(const Point3D& p1, const Point3D& p2); // Determine if p1 and p2 are in the same region
inline bool in_same_subregion(const float& x1, const float& x2);  // Utility function to determine if x1 and x2 are in the same subregion
inline void print_point(const Point3D& p);                        // Print a point
inline void print_region(const Region& r);                        // Print a region

int main()
{
    // Create some Point3D instances
    Point3D p1 = {1.0, 2.0, 3.0};
    Point3D p2 = {-1.0, 2.0, 3.0};
    Point3D p3 = {1.0, -2.0, -3.0};
    Point3D p4 = {4.0, 5.0, 6.0};

    // Print the points
    print_point(p1);
    print_point(p2);
    print_point(p3);
    print_point(p4);

    // Calculate and print regions
    Region r1 = region(p1);
    Region r2 = region(p2);
    Region r3 = region(p3);
    Region r4 = region(p4);

    print_region(r1);
    print_region(r2);
    print_region(r3);
    print_region(r4);

    // Test zero_distance and compare functions
    float dist1 = zero_distance(p1);
    float dist2 = zero_distance(p2);
    bool isP1Greater = compare(p1, p2);

    std::cout << "Distance from p1 to origin: " << dist1 << std::endl;
    std::cout << "Distance from p2 to origin: " << dist2 << std::endl;
    std::cout << "Is p1's distance greater than p2's? " << (isP1Greater ? "Yes" : "No") << std::endl;

    // Test in_same_region function
    bool inSameRegion1and2 = in_same_region(p1, p2);
    bool inSameRegion1and4 = in_same_region(p1, p4);

    std::cout << "Are p1 and p2 in the same region? " << (inSameRegion1and2 ? "Yes" : "No") << std::endl;
    std::cout << "Are p1 and p4 in the same region? " << (inSameRegion1and4 ? "Yes" : "No") << std::endl;
    return 0;
}


inline float zero_distance(const Point3D& p1)
{
    return distance(p1, { 0, 0, 0 });
}

inline float distance(const Point3D&  p1, const Point3D& p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

inline bool compare(const Point3D& p1, const Point3D& p2)
{
    return zero_distance(p1) > zero_distance(p2) ? true : false;
}

inline Region region(const Point3D& p)
{
    if(p.x > 0 && p.y > 0 && p.z > 0){
        return First;
    }else if(p.x < 0 && p.y > 0 && p.z > 0){
        return Second;
    }else if(p.x < 0 && p.y < 0 && p.z > 0){
        return Third;
    }else if(p.x > 0 && p.y < 0 && p.z > 0){
        return Fourth;
    }else if(p.x > 0 && p.y > 0 && p.z < 0){
        return Fifth;
    }else if(p.x < 0 && p.y > 0 && p.z < 0){
        return Sixth;
    }else if(p.x < 0 && p.y < 0 && p.z < 0){
        return Seventh;
    }else if(p.x > 0 && p.y < 0 && p.z < 0){
        return Eighth;
    }else return None;
}

inline bool in_same_region(const Point3D& p1, const Point3D& p2)
{
    if(in_same_subregion(p1.x, p2.x) && in_same_subregion(p1.x, p2.y) &&
        in_same_subregion(p1.z, p2.z)){
            return true;
        }else return false;
}

inline bool in_same_subregion(const float& x1, const float& x2)
{
    return x1 * x2 > 0 ? true : false;
}

inline void print_point(const Point3D& p)
{
    std::cout << "P" << "(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
}

inline void print_region(const Region& r)
{
    (r == None) ? std::cout << "None, the point is not in a region!" << std::endl : std::cout << r << std::endl;
}
