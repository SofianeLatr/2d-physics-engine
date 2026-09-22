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

                collisionPairs.push_back(std::make_pair(A, B));
            }
        }
    }
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