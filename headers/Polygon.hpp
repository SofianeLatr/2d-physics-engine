#ifndef POLYGON_HPP
#define POLYGON_HPP

#include "Body.hpp"
#include <vector>

class Polygon : public Body {
public:
    std::vector<Vec2> points;

    Polygon(Vec2 pos, Vec2 size, float angle, float mass, bool isStatic);
    virtual ~Polygon();

    virtual const std::vector<Vec2>& getPoints() {
        return points;
    }
};

class Rect : public Polygon {
public:
    Vec2 size;

    Rect(Vec2 pos, Vec2 size, float angle, float mass, bool isStatic);
    ~Rect() override;

    const std::vector<Vec2>& updatePoints();
    const std::vector<Vec2>& getPoints() override {
        return updatePoints();
    }

    float getInertia() {
        return (1.0f / 12.0f) * mass *
               (size.x * size.x + size.y * size.y);
    }
};

#endif
