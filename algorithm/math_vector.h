#ifndef MATH_VECTOR_H
#define MATH_VECTOR_H

#include <math.h>

#define 	PI			3.1415926535898f
#define 	RAD2DEG 	(180.0f / 3.1415926535898f)
#define		DEG2RAD		(3.1415926535898f / 180.f)

// 2D Vector class for planar coordinates, velocities, etc.
class Vector2f {
public:
    float x;
    float y;

    // Constructors
    Vector2f() : x(0), y(0) {}
    Vector2f(float x_val, float y_val) : x(x_val), y(y_val) {}

    // Operator overloading: vector addition & subtraction
    Vector2f operator+(const Vector2f& other) const {
        return Vector2f(x + other.x, y + other.y);
    }
    Vector2f operator-(const Vector2f& other) const {
        return Vector2f(x - other.x, y - other.y);
    }

    // Operator overloading: scalar multiplication & division
    Vector2f operator*(float scalar) const {
        return Vector2f(x * scalar, y * scalar);
    }
    Vector2f operator/(float scalar) const {
        return Vector2f(x / scalar, y / scalar);
    }

    // Common operation: dot product
    float dot(const Vector2f& other) const {
        return x * other.x + y * other.y;
    }

    // Common operation: vector magnitude (length)
    float norm() const {
        return sqrt(x * x + y * y);
    }

    // Common operation: normalize to unit vector
    Vector2f normalized() const {
        float n = norm();
        if (n < 1e-6f) return Vector2f(0, 0);
        return *this / n;
    }

    // Common operation: planar rotation (unit: radians)
    Vector2f rotated(float rad) const {
        float cos_r = cos(rad);
        float sin_r = sin(rad);
        return Vector2f(
            x * cos_r - y * sin_r,
            x * sin_r + y * cos_r
        );
    }
	
	//Get angle of vector with x axis(-PI ~ PI)
	float angle(){
		return atan2(y, x);
	}
};

// 3D Vector class for spatial coordinates, angular velocities, forces, etc.
class Vector3f {
public:
    float x;
    float y;
    float z;

    // Constructors
    Vector3f() : x(0), y(0), z(0) {}
    Vector3f(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}

    // Operator overloading: vector addition & subtraction
    Vector3f operator+(const Vector3f& other) const {
        return Vector3f(x + other.x, y + other.y, z + other.z);
    }
    Vector3f operator-(const Vector3f& other) const {
        return Vector3f(x - other.x, y - other.y, z - other.z);
    }

    // Operator overloading: scalar multiplication & division
    Vector3f operator*(float scalar) const {
        return Vector3f(x * scalar, y * scalar, z * scalar);
    }
    Vector3f operator/(float scalar) const {
        return Vector3f(x / scalar, y / scalar, z / scalar);
    }

    // Common operation: dot product
    float dot(const Vector3f& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // Common operation: cross product
    Vector3f cross(const Vector3f& other) const {
        return Vector3f(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    // Common operation: vector magnitude (length)
    float norm() const {
        return sqrt(x * x + y * y + z * z);
    }

    // Common operation: normalize to unit vector
    Vector3f normalized() const {
        float n = norm();
        if (n < 1e-6f) return Vector3f(0, 0, 0);
        return *this / n;
    }
};

#endif // MATH_VECTOR_H
