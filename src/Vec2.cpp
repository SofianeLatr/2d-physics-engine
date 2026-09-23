#include "Vec2.hpp"

Vec2::~Vec2() {};
Vec2::Vec2() : x(0), y(0) {};
Vec2::Vec2(float x, float y) : x(x), y(y) {};

Vec2& Vec2::normalize() {
    float length = this->length();
    if (length > 0) {
        x /= length;
        y /= length;
    }
    return *this;
}
