#ifndef COLLISIONDETECTION_HPP
#define COLLISIONDETECTION_HPP

#include "Vec2.hpp"
#include "Polygon.hpp"
#include "BroadPhaseMath.hpp"

#include <vector>
#include <algorithm>
#include <utility>

struct CollisionInfo {
    const Polygon* A;
    const Polygon* B;

    float overlap;
    Vec2 normal;

    int referenceEdge;
    bool referenceIsA;
};

CollisionInfo SAT(const Polygon& A, const Polygon& B)
{
    const std::vector<Vec2>& pointsA = const_cast<Polygon&>(A).getPoints();
    const std::vector<Vec2>& pointsB = const_cast<Polygon&>(B).getPoints();

    float minOverlap = 0;
    Vec2 normal;
    int referenceEdge = 0;
    bool referenceIsA = true;
    bool first = true;

    for (int i = 0; i < pointsA.size(); i++)
    {
        Vec2 edge = pointsA[(i + 1) % pointsA.size()] - pointsA[i];
        Vec2 axis = edge.normal();
        axis.normalize();

        float minA = pointsA[0] * axis;
        float maxA = minA;

        for (const Vec2& point : pointsA)
        {
            float projection = point * axis;

            minA = std::min(minA, projection);
            maxA = std::max(maxA, projection);
        }

        float minB = pointsB[0] * axis;
        float maxB = minB;

        for (const Vec2& point : pointsB)
        {
            float projection = point * axis;

            minB = std::min(minB, projection);
            maxB = std::max(maxB, projection);
        }

        if (maxA < minB || maxB < minA)
        {
            return {
                &A,
                &B,
                0,
                Vec2(0, 0),
                -1,
                true
            };
        }

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

    for (int i = 0; i < pointsB.size(); i++)
    {
        Vec2 edge = pointsB[(i + 1) % pointsB.size()] - pointsB[i];
        Vec2 axis = edge.normal();
        axis.normalize();

        float minA = pointsA[0] * axis;
        float maxA = minA;

        for (const Vec2& point : pointsA)
        {
            float projection = point * axis;

            minA = std::min(minA, projection);
            maxA = std::max(maxA, projection);
        }

        float minB = pointsB[0] * axis;
        float maxB = minB;

        for (const Vec2& point : pointsB)
        {
            float projection = point * axis;

            minB = std::min(minB, projection);
            maxB = std::max(maxB, projection);
        }

        if (maxA < minB || maxB < minA)
        {
            return {
                &A,
                &B,
                0,
                Vec2(0, 0),
                -1,
                false
            };
        }

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

    Vec2 direction = B.pos - A.pos;

    if (direction * normal < 0)
        normal *= -1;

    return {
        &A,
        &B,
        minOverlap,
        normal,
        referenceEdge,
        referenceIsA
    };
}

std::pair<int, std::vector<Vec2>> getContactPoints(
    const CollisionInfo& info)
{
    const std::vector<Vec2>& reference =
        info.referenceIsA
            ? const_cast<Polygon*>(info.A)->Polygon::getPoints()
            : const_cast<Polygon*>(info.B)->Polygon::getPoints();

    const std::vector<Vec2>& incident =
        info.referenceIsA
            ? const_cast<Polygon*>(info.B)->Polygon::getPoints()
            : const_cast<Polygon*>(info.A)->Polygon::getPoints();

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
        Vec2 edgeVector =
            incident[(i + 1) % incident.size()] - incident[i];

        Vec2 normal = edgeVector.normal();
        normal.normalize();

        float dot = normal * refNormal;

        if (dot < minDot)
        {
            minDot = dot;
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
        float t = (offset1 - maxOffset) /
                  (offset1 - offset2);

        i1 = i1 + (i2 - i1) * t;
    }

    if (offset2 > maxOffset)
    {
        float t = (offset2 - maxOffset) /
                  (offset2 - offset1);

        i2 = i2 + (i1 - i2) * t;
    }

    std::vector<Vec2> contacts;

    float faceDistance = r1 * refNormal;

    float depth1 = i1 * refNormal - faceDistance;
    float depth2 = i2 * refNormal - faceDistance;

    if (depth1 <= info.overlap)
        contacts.push_back(i1);

    if (depth2 <= info.overlap && contacts.size() < 2)
        contacts.push_back(i2);

    return {
        static_cast<int>(contacts.size()),
        contacts
    };
}

AABB getAABB(const std::vector<Vec2>& polygon)
{
    AABB box;

    box.min = polygon[0];
    box.max = polygon[0];

    for (int i = 1; i < polygon.size(); i++)
    {
        box.min.x = std::min(box.min.x, polygon[i].x);
        box.min.y = std::min(box.min.y, polygon[i].y);

        box.max.x = std::max(box.max.x, polygon[i].x);
        box.max.y = std::max(box.max.y, polygon[i].y);
    }

    return box;
}

bool AABBcollision(const AABB& A, const AABB& B)
{
    if (A.max.x < B.min.x || A.min.x > B.max.x)
        return false;

    if (A.max.y < B.min.y || A.min.y > B.max.y)
        return false;

    return true;
}

#endif