#include "Simulation.hpp"



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