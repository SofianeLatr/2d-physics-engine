#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "CollisionDetection.hpp"
#include "BroadPhaseMath.hpp"
#include "Polygon.hpp"
#include <iostream>
class Simulation {
public:

    std::vector<Body*> bodies;
    std::vector<std::pair<Body*, Body*>> collisionPairs;

    float dt;
    int iterations = 10;

    Simulation();
    ~Simulation();

    void addBody(Body* body);
    void removeBody(Body* body);
    
    void simulate(float dt);
    void checkCollisions();
    void solveConstrains();
    void applyForces();
    void updatePositions();

    int main();
};

#endif