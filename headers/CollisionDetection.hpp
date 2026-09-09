#ifndef COLLISION_DETECTION_HPP
#define COLLISION_DETECTION_HPP

#include "Vec2.hpp"
#include "Polygon.hpp"
#include "BroadPhaseMath.hpp"
#include <array>
#include <algorithm>
#include <limits>

struct CollisionInfo {
    float overlap;
    Vec2 normal;
};


template<int verticesA, int verticesB>
CollisionInfo SAT(const std::array<Vec2, verticesA>& A, const std::array<Vec2, verticesB>& B)
{
    float minOverlap = 0;
    Vec2 normal;
    bool first = true;

    for (int i = 0; i < verticesA; i++)
    {
        Vec2 edge = A[(i + 1) % verticesA] - A[i];
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

        float overlap = std::min(maxA, maxB) -
                        std::max(minA, minB);

        if (first || overlap < minOverlap)
        {
            minOverlap = overlap;
            normal = axis;
            first = false;
        }
    }

    for (int i = 0; i < verticesB; i++)
    {
        Vec2 edge = B[(i + 1) % verticesB] - B[i];
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

        float overlap = std::min(maxA, maxB) -
                        std::max(minA, minB);

        if (first || overlap < minOverlap)
        {
            minOverlap = overlap;
            normal = axis;
            first = false;
        }
    }

    Vec2 direction = B[0] - A[0];

    if (direction * normal < 0)
        normal *= -1;

    return {minOverlap, normal};
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