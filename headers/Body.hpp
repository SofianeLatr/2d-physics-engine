#ifndef Body_h
#define Body_h

#include "Vec2.hpp"
#include <stdexcept>

class Shape{
public:
    Vec2 pos;
    float angle;
    bool isStatic = false;
    bool polygon = true;

    virtual ~Shape() = default;
    Shape(Vec2 pos, float angle, bool isStatic) : pos(pos), angle(angle), isStatic(isStatic) {};

};

class Body : public Shape {
public:

    Vec2 vel;
    float angVel;

    float mass;
    float invMass;
    float inertia;
    float invInertia;

    Vec2 force;
    float torque;

    ~Body() override = default;
    Body(Vec2 pos, float angle, float mass, bool isStatic) : Shape(pos, angle, isStatic), mass(mass){
        if (isStatic) {
            this->mass = 0;
            invMass = 0;
            inertia = 0;
            invInertia = 0;
        } else {
            if (!(mass > 0))
                throw std::invalid_argument("Dynamic body mass must be positive");
            invMass = 1.0f / mass;
            inertia = mass;
            invInertia = 1.0f / inertia;
        }
        vel = Vec2(0, 0);
        angVel = 0;
        force = Vec2(0, 0);
        torque = 0;
    }

    void applyForce(const Vec2& f){
        force += f;
    }
    void applyTorque(float t){
        torque += t;
    }
};

#endif
