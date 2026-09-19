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

void Simulation::checkCollisions()
{
    for (int i = 0; i < bodies.size(); i++)
    {
        for (int j = i + 1; j < bodies.size(); j++)
        {
            Body* A = bodies[i];
            Body* B = bodies[j];

            if (A->isStatic && B->isStatic)
                continue;

                
            Polygon<4>* polyA = static_cast<Polygon<4>*>(A);
            Polygon<4>* polyB = static_cast<Polygon<4>*>(B);

            auto pointsA = polyA->getPoints();
            auto pointsB = polyB->getPoints();

            AABB boxA = getAABB<4>(pointsA);
            AABB boxB = getAABB<4>(pointsB);

            if (!AABBcollision(boxA, boxB))
                continue;

            CollisionInfo info = SAT<4, 4>(pointsA, pointsB);

            if (info.overlap > 0)
            {
                std::cout << "Collision detected!\n";

                auto contacts =
                    getContactPoints<4, 4>(pointsA, pointsB, info);

                std::cout << "Contacts: "
                          << contacts.first << "\n";
            }
        }
    }
}

int Simulation::main() {
    std::array<Vec2, 4> A = {
    Vec2(-1, -1),
    Vec2(1, -1),
    Vec2(1, 1),
    Vec2(-1, 1)
    };

    std::array<Vec2, 4> B = {
        Vec2(0, -1),
        Vec2(2, -1),
        Vec2(2, 1),
        Vec2(0, 1)
    };

    AABB boxA = getAABB<A.size()>(A);
    AABB boxB = getAABB<B.size()>(B);

    if (AABBcollision(boxA, boxB))
    {
        std::cout << "AABBs are colliding, proceeding to SAT test..." << std::endl;
    }
    CollisionInfo info = SAT<A.size(), B.size()>(A, B);

    if (info.overlap > 0) {
        std::cout << "Collision detected!" << std::endl;
        std::cout << "Overlap: " << info.overlap << std::endl;
        std::cout << "Normal: (" << info.normal.x << ", " << info.normal.y << ")" << std::endl;
        std::cout << "Reference Edge: " << info.referenceEdge << std::endl;
        std::cout << "Reference is A: " << (info.referenceIsA ? "true" : "false") << std::endl;
        
        auto contactPoints = getContactPoints<A.size(), B.size()>(A, B, info);
        std::cout << "Number of contact points: " << contactPoints.first << std::endl;
        for (int i = 0; i < contactPoints.first; ++i) {
            std::cout << "Contact Point " << i + 1 << ": (" << contactPoints.second[i].x << ", " << contactPoints.second[i].y << ")" << std::endl;
        }   
    } else {
        std::cout << "No collision detected." << std::endl;
    }

    std::cin.get(); // Wait for user input before closing the console window

    return 0;
}