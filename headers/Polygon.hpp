#ifndef Polygon_h
#define Polygon_h

#include "Body.hpp"
#include <array>

template<int N>
class Polygon : public Body {
public:
    static const int verticies = N;
    std::array<Vec2, N> points;

    const std::array<Vec2, N>& getPoints();
    
    Polygon(Vec2 pos, Vec2 size, float angle, float mass, bool isStatic);
    ~Polygon();
};

class Rect : public Polygon<4> {
public:
    std::array<Vec2, 4> points;
    Vec2 size;

    ~Rect(){};
    Rect(Vec2 pos, Vec2 size, float angle, float mass, bool isStatic);    

    const std::array<Vec2, 4>& getPoints();
    const float getInertia() { return (1.0f / 12.0f) * mass * (size.x * size.x + size.y * size.y); };
};

#endif