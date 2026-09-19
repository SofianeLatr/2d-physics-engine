#include "Polygon.hpp"

template<int N>
Polygon<N>::~Polygon() {};
template<int N>
Polygon<N>::Polygon(Vec2 pos, Vec2 size, float angle, float mass, bool isStatic) : Body(pos, angle, mass, isStatic) {};
Rect::Rect(Vec2 pos, Vec2 size, float angle, float mass, bool isStatic) : Polygon<4>(pos, size, angle, mass, isStatic), size(size) {
    this->getPoints();
    inertia = this->getInertia();
};

template<int N>
const std::array<Vec2, N>& Polygon<N>::getPoints()  {
    return points;
}

const std::array<Vec2, 4>& Rect::getPoints()  {
    points[0] = Vec2(-size.x / 2,-size.y / 2);
    points[1] = Vec2(+size.x / 2,-size.y / 2);
    points[2] = Vec2(+size.x / 2,+size.y / 2);
    points[3] = Vec2(-size.x / 2,+size.y / 2);
    

    for(int i = 0; i < 4; i++) {
        points[i] = Vec2(points[i].x * cos(angle) - points[i].y * sin(angle), 
                         points[i].x * sin(angle) + points[i].y * cos(angle));
    }

    points[0] += Vec2(pos.x,pos.y);
    points[1] += Vec2(pos.x,pos.y);
    points[2] += Vec2(pos.x,pos.y);
    points[3] += Vec2(pos.x,pos.y);    



    return points;
}