include "CollisionDetection.hpp"
#include "BroadPhaseMath.hpp"J
#include <iostream>

int main() {
    std::array<Vec2, 4> A = {Vec2(0, 0), Vec2(1, 0), Vec2(1, 1), Vec2(0, 1)};
    std::array<Vec2, 4> B = {Vec2(0.5, 0.5), Vec2(1.5, 0.5), Vec2(1.5, 1.5), Vec2(0.5, 1.5)};

    CollisionInfo info = SAT<A.size(), B.size()>(A, B);

    if (info.overlap > 0) {
        std::cout << "Collision detected!" << std::endl;
        std::cout << "Overlap: " << info.overlap << std::endl;
        std::cout << "Normal: (" << info.normal.x << ", " << info.normal.y << ")" << std::endl;
        std::cout << "Reference Edge: " << info.referenceEdge << std::endl;
        std::cout << "Reference is A: " << (info.referenceIsA ? "true" : "false") << std::endl;
    } else {
        std::cout << "No collision detected." << std::endl;
    }

    return 0;
}