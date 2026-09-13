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
    const auto& incident  = info.referenceIsA ? B : A;

    int referenceEdge = info.referenceEdge;

    Vec2 r1 = reference[referenceEdge];
    Vec2 r2 = reference[(referenceEdge + 1) % reference.size()];

    Vec2 referenceNormal = info.normal;

    // Find incident edge
    int incidentEdge = 0;
    float minDot = 999999;

    for (int i = 0; i < incident.size(); i++)
    {
        Vec2 edge = incident[(i + 1) % incident.size()] - incident[i];
        Vec2 edgeNormal = edge.normal();

        float d = edgeNormal * referenceNormal;

        if (d < minDot)
        {
            minDot = d;
            incidentEdge = i;
        }
    }

    Vec2 i1 = incident[incidentEdge];
    Vec2 i2 = incident[(incidentEdge + 1) % incident.size()];

    // Reference edge direction
    Vec2 side = r2 - r1;
    side.normalize();

    // Clip against first side
    float d1 = (i1 - r1) * side;
    float d2 = (i2 - r1) * side;

    if (d1 < 0 && d2 < 0)
        return {0, {}};

    if (d1 < 0)
        i1 = i1 + (i2 - i1) * (-d1 / (d2 - d1));

    if (d2 < 0)
        i2 = i2 + (i1 - i2) * (-d2 / (d1 - d2));

    // Clip against second side
    float maxSide = (r2 - r1) * side;

    d1 = (i1 - r1) * side;
    d2 = (i2 - r1) * side;

    if (d1 > maxSide && d2 > maxSide)
        return {0, {}};

    if (d1 > maxSide)
        i1 = i1 + (i2 - i1) * ((maxSide - d1) / (d2 - d1));

    if (d2 > maxSide)
        i2 = i2 + (i1 - i2) * ((maxSide - d2) / (d1 - d2));

    // Keep only points behind reference face
    float faceDistance = r1 * referenceNormal;

    float depth1 = i1 * referenceNormal - faceDistance;
    float depth2 = i2 * referenceNormal - faceDistance;

    std::array<Vec2, 2> contacts;
    int count = 0;

    if (depth1 <= 0)
        contacts[count++] = i1;

    if (depth2 <= 0)
        contacts[count++] = i2;

    return {count, contacts};
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