#include "Polygon.hpp"
#include <vector>

Polygon::~Polygon() {};
Polygon::Polygon(Vec2 pos, Vec2 size, float angle, float mass, bool isStatic) : Body(pos, angle, mass, isStatic) {};
Rect::Rect(Vec2 pos, Vec2 size, float angle, float mass, bool isStatic) : Polygon(pos, size, angle, mass, isStatic), size(size) {
    this->getPoints();
    inertia = this->getInertia();
};

const std::vector<Vec2>& Rect::updatePoints()
{
    if (points.empty())
    {
        points.push_back(Vec2(-size.x / 2, -size.y / 2));
        points.push_back(Vec2( size.x / 2, -size.y / 2));
        points.push_back(Vec2( size.x / 2,  size.y / 2));
        points.push_back(Vec2(-size.x / 2,  size.y / 2));
    }

    for (int i = 0; i < 4; i++)
    {
        Vec2 local = points[i];

        points[i] = Vec2(
            local.x * cos(angle) - local.y * sin(angle),
            local.x * sin(angle) + local.y * cos(angle)
        );

        points[i] += pos;
    }

    return points;
}