#include "Polygon.hpp"
#include <vector>

Polygon::~Polygon() {};
Polygon::Polygon(Vec2 pos, Vec2 size, float angle, float mass, bool isStatic) : Body(pos, angle, mass, isStatic) {};
Rect::~Rect() = default;
Rect::Rect(Vec2 pos, Vec2 size, float angle, float mass, bool isStatic) : Polygon(pos, size, angle, mass, isStatic), size(size) {
    this->getPoints();
    if (!isStatic) {
        inertia = this->getInertia();
        invInertia = inertia > 0 ? 1.0f / inertia : 0.0f;
    }
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
        // Rebuild from local-space vertices so repeated queries never transform
        // already transformed world-space points.
        const Vec2 localPoints[4] = {
            Vec2(-size.x / 2, -size.y / 2), Vec2(size.x / 2, -size.y / 2),
            Vec2(size.x / 2, size.y / 2), Vec2(-size.x / 2, size.y / 2)
        };
        Vec2 local = localPoints[i];

        points[i] = Vec2(
            local.x * cos(angle) - local.y * sin(angle),
            local.x * sin(angle) + local.y * cos(angle)
        );

        points[i] = points[i] + pos;
    }

    return points;
}
