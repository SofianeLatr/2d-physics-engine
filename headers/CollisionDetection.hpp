#ifndef COLLISION_DETECTION_HPP
#define COLLISION_DETECTION_HPP

#include "Vec2.hpp"
#include <array>
#include <algorithm>
#include <limits>

struct CollisionInfo {
    float overlap;
    Vec2 normal;
};

struct AABB {
    Vec2 min;
    Vec2 max;
};

template<int verticesA, int verticesB>
CollisionInfo SAT(const std::array<Vec2, verticesA>& A, const std::array<Vec2, verticesB>& B){
    float minOverlap = 0;
    Vec2 normal;
    bool first = true;

    for (int shape = 0; shape < 2; shape++)
    {
        const auto& points = (shape == 0) ? A : B;
    
        for (int i = 0; i < std::max(verticesA, verticesB); i++)
        {
            Vec2 edge = points[(i + 1) % std::max(verticesA, verticesB)] - points[i];
            Vec2 axis = edge.normal();

            float minA = A[0] * axis;
            float maxA = minA;

            for (const Vec2& point : A)
            {
                float p = point * axis;
                minA = std::min(minA, p);
                maxA = std::max(maxA, p);
            }
            float minB = B[0] * axis;
            float maxB = minB;

            for (const Vec2& point : B)
            {
                float p = point * axis;
                minB = std::min(minB, p);
                maxB = std::max(maxB, p);
            }
            if (maxA < minB || maxB < minA)
                return {0, Vec2(0, 0)};
            float overlap = std::min(maxA, maxB) - std::max(minA, minB);

            if (maxA < maxB && minA < minB)
                overlap = maxA - minB;
            else if (maxB < maxA && minB < minA)
                overlap = maxB - minA;

            if (first || overlap < minOverlap)
            {
                minOverlap = overlap;
                normal = axis;
                first = false;
            }
        }
    }

    Vec2 direction = B[0] - A[0];

    if (direction * normal < 0)
        normal *= -1;

    CollisionInfo result{};
    result.overlap = minOverlap;
    result.normal = normal;
    return result;
}



template<int N>
AABB getAABB(const Polygon<N>& polygon)
{
    AABB box;
    box.min = polygon.points[0];
    box.max = polygon.points[0];

    for(int i = 1; i < N; i++) {
        box.min.x = std::min(box.min.x, polygon.points[i].x);
        box.min.y = std::min(box.min.y, polygon.points[i].y);
        box.max.x = std::max(box.max.x, polygon.points[i].x);
        box.max.y = std::max(box.max.y, polygon.points[i].y);
    }

    return box;
}

bool AABBcollision(const AABB& A, const AABB& B)
{
    if(A.max.x < B.min.x || A.min.x > B.max.x)
        return false;

    if(A.max.y < B.min.y || A.min.y > B.max.y)
        return false;

    return true;
}

#endif