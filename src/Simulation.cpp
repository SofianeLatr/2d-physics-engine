#include "Simulation.hpp"

Simulation::~Simulation() {}

Simulation::Simulation() {}

void Simulation::addBody(Body* body) {
    bodies.push_back(body);
}

void Simulation::simulate(float dt) {
    this->dt = dt;
    this->dt /= (float)iterations;

    for(int i = 0; i < iterations; i++) {
        checkCollisions();
        solveConstrains();
        applyForces();
        updatePositions();
    }

    for(auto body : bodies) {
        body->force = Vec2(0,0);
        body->torque = 0;
    }
}

void Simulation::checkCollisions() {
    for(int i = 0; i < bodies.size(); i++) {
        for(int j = i + 1; j < bodies.size(); j++) {

            Body* A = bodies[i];
            Body* B = bodies[j];

            if(A->isStatic && B->isStatic)
                continue;

            Polygon* polyA = static_cast<Polygon*>(A);
            Polygon* polyB = static_cast<Polygon*>(B);

            const std::vector<Vec2>& pointsA = polyA->getPoints();
            const std::vector<Vec2>& pointsB = polyB->getPoints();

            AABB boxA = getAABB(pointsA);
            AABB boxB = getAABB(pointsB);

            if(!AABBcollision(boxA, boxB))
                continue;

            CollisionInfo info = SAT(*polyA, *polyB);

            if(info.overlap > 0) {
                auto contacts = getContactPoints(info);

                collisionPairs.push_back(info);
            }
        }
    }
}

void Simulation::solveConstrains() {
    for(auto info : collisionPairs) {
        Body* A = info.A;
        Body* B = info.B;

        if(info.overlap > 0) {
            auto contacts = getContactPoints(info);

            for(int i = 0; i < contacts.first; i++) {
                Vec2 contactPoint = contacts.second[i];

                Vec2 rA = contactPoint - A->pos;
                Vec2 rB = contactPoint - B->pos;

                Vec2 relativeVel =
                    (B->vel + Vec2(-B->angVel * rB.y, B->angVel * rB.x)) -
                    (A->vel + Vec2(-A->angVel * rA.y, A->angVel * rA.x));

                float velAlongNormal = relativeVel * info.normal;

                if(velAlongNormal > 0)
                    continue;

                float e = 0.5f;

                float j = -(1 + e) * velAlongNormal;

                j /= A->invMass + B->invMass +
                     (rA.cross(info.normal) * rA.cross(info.normal)) * A->invInertia +
                     (rB.cross(info.normal) * rB.cross(info.normal)) * B->invInertia;

                Vec2 impulse = info.normal * j;

                A->vel -= impulse * A->invMass;
                A->angVel -= rA.cross(impulse) * A->invInertia;

                B->vel += impulse * B->invMass;
                B->angVel += rB.cross(impulse) * B->invInertia;
            }
        }
    }

    collisionPairs.clear();
}

int Simulation::main() {

    std::vector<Vec2> A = {
        Vec2(-1, -1),
        Vec2(1, -1),
        Vec2(1, 1),
        Vec2(-1, 1)
    };

    std::vector<Vec2> B = {
        Vec2(0, -1),
        Vec2(2, -1),
        Vec2(2, 1),
        Vec2(0, 1)
    };

    std::cin.get();

    return 0;
}