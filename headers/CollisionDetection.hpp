#ifndef ColisionDetection_HPP
#define ColisionDetection_HPP

#include "Vec2.hpp"
#include "Polygon.hpp"
#include "BroadPhaseMath.hpp"
#include <array>
#include <algorithm>
#include <limits>

struct CollisionInfo {
    float overlap;
    Vec2 normal;

    int referenceEdge;
    bool referenceIsA;
};


template<int verticesA, int verticesB>
CollisionInfo SAT(
    const std::array<Vec2, verticesA>& A,
    const std::array<Vec2, verticesB>& B)
{
    float minOverlap = 0;
    Vec2 normal;
    int referenceEdge = 0;
    bool referenceIsA = true;
    bool first = true;

    // Check A's edges
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
            return {0, Vec2(0, 0), -1, true};

        float overlap = std::min(maxA, maxB) -
                        std::max(minA, minB);

        if (first || overlap < minOverlap)
        {
            minOverlap = overlap;
            normal = axis;
            referenceEdge = i;
            referenceIsA = true;
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
            return {0, Vec2(0, 0), -1, false};

        float overlap = std::min(maxA, maxB) -
                        std::max(minA, minB);

        if (first || overlap < minOverlap)
        {
            minOverlap = overlap;
            normal = axis;
            referenceEdge = i;
            referenceIsA = false;
            first = false;
        }
    }

    Vec2 direction = B[0] - A[0];

    if (direction * normal < 0)
        normal *= -1;

    return {
        minOverlap,
        normal,
        referenceEdge,
        referenceIsA
    };
}

template<int verticesA, int verticesB>
std::pair<int, std::array<Vec2, 2>> getContactPoints(
    const std::array<Vec2, verticesA>& A,
    const std::array<Vec2, verticesB>& B,
    const CollisionInfo& info)
{
    const auto& reference = info.referenceIsA ? A : B;
    const auto& incident = info.referenceIsA ? B : A;

    int edge = info.referenceEdge;

    Vec2 r1 = reference[edge];
    Vec2 r2 = reference[(edge + 1) % reference.size()];

    Vec2 refNormal = info.normal;

    Vec2 side = r2 - r1;
    side.normalize();

    int incidentEdge = 0;
    float minDot = 999999;

    for (int i = 0; i < incident.size(); i++)
    {
        Vec2 e = incident[(i + 1) % incident.size()] - incident[i];
        Vec2 n = e.normal();
        float d = n * refNormal;

        if (d < minDot)
        {
            minDot = d;
            incidentEdge = i;
        }
    }

    Vec2 i1 = incident[incidentEdge];
    Vec2 i2 = incident[(incidentEdge + 1) % incident.size()];

    float offset1 = (i1 - r1) * side;
    float offset2 = (i2 - r1) * side;

    if (offset1 < 0)
    {
        float t = offset1 / (offset1 - offset2);
        i1 = i1 + (i2 - i1) * t;
    }

    if (offset2 < 0)
    {
        float t = offset2 / (offset2 - offset1);
        i2 = i2 + (i1 - i2) * t;
    }

    float maxOffset = (r2 - r1) * side;

    offset1 = (i1 - r1) * side;
    offset2 = (i2 - r1) * side;

    if (offset1 > maxOffset)
    {
        float t = (offset1 - maxOffset) / (offset1 - offset2);
        i1 = i1 + (i2 - i1) * t;
    }

    if (offset2 > maxOffset)
    {
        float t = (offset2 - maxOffset) / (offset2 - offset1);
        i2 = i2 + (i1 - i2) * t;
    }

    std::array<Vec2, 2> contacts;
    int count = 0;

    float faceDistance = r1 * refNormal;

    float depth1 = i1 * refNormal - faceDistance;
    float depth2 = i2 * refNormal - faceDistance;

    if (depth1 <= info.overlap)
        contacts[count++] = i1;

    if (depth2 <= info.overlap && count < 2)
        contacts[count++] = i2;

    return {count, contacts};
}

template<int N>
AABB getAABB(const std::array<Vec2, N>& polygon)
{
    AABB box;
    box.min = polygon[0];
    box.max = polygon[0];

    for(int i = 1; i < N; i++) {
        box.min.x = std::min(box.min.x, polygon[i].x);
        box.min.y = std::min(box.min.y, polygon[i].y);
        box.max.x = std::max(box.max.x, polygon[i].x);
        box.max.y = std::max(box.max.y, polygon[i].y);
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