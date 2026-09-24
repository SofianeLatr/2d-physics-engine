#include "Simulation.hpp"

Simulation::~Simulation() {}

Simulation::Simulation() {}

void Simulation::addBody(Body* body) {
    if (body != nullptr)
        bodies.push_back(body);
}

void Simulation::removeBody(Body* body) {
    bodies.erase(std::remove(bodies.begin(), bodies.end(), body), bodies.end());
    collisionPairs.erase(
        std::remove_if(collisionPairs.begin(), collisionPairs.end(),
            [body](const CollisionInfo& info) {
                return info.A == body || info.B == body;
            }),
        collisionPairs.end());
}

void Simulation::simulate(float dt) {
    if (iterations <= 0 || dt <= 0)
        return;
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
    collisionPairs.clear();
    for(int i = 0; i < bodies.size(); i++) {
        for(int j = i + 1; j < bodies.size(); j++) {

            Body* A = bodies[i];
            Body* B = bodies[j];

            if(A->isStatic && B->isStatic)
                continue;

            Polygon* polyA = dynamic_cast<Polygon*>(A);
            Polygon* polyB = dynamic_cast<Polygon*>(B);
            if (polyA == nullptr || polyB == nullptr)
                continue;

            const std::vector<Vec2>& pointsA = polyA->getPoints();
            const std::vector<Vec2>& pointsB = polyB->getPoints();

            AABB boxA = getAABB(pointsA);
            AABB boxB = getAABB(pointsB);

            if(!AABBcollision(boxA, boxB))
                continue;

            CollisionInfo info = SAT(*polyA, *polyB);

            if(info.overlap > 0) {
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

            const float penetrationSlop = 0.01f;
            const float correctionPercent = 0.8f;
            float inverseMassSum = A->invMass + B->invMass;
            if (inverseMassSum > 0.0f && info.overlap > penetrationSlop) {
                float correctionDepth =
                    (info.overlap - penetrationSlop) * correctionPercent / inverseMassSum;
                Vec2 correction = info.normal * correctionDepth;
                A->pos -= correction * A->invMass;
                B->pos += correction * B->invMass;
            }
        }
    }

    collisionPairs.clear();
}

void Simulation::applyForces() {

    for(auto body : bodies) {
        if(body->isStatic)
            continue;

        //gravity
        Vec2 gravity(0, 9.81f);
        Vec2 acceleration = body->force * body->invMass + gravity;
        float angAcceleration = body->torque * body->invInertia;

        body->vel += acceleration * dt;
        body->angVel += angAcceleration * dt;
    }
}

void Simulation::updatePositions() {
    for(auto body : bodies) {
        if(body->isStatic)
            continue;

        body->pos += body->vel * dt;
        body->angle += body->angVel * dt;
    }
}

int Simulation::runDemo() {

    // Positive y points down in this engine, so the ground sits below the
    // falling rectangles.
    Rect first(Vec2(-1.5f, 0.0f), Vec2(2.0f, 1.0f), -0.35f, 2.0f, false);
    Rect second(Vec2(1.5f, -2.0f), Vec2(2.0f, 1.0f), 0.40f, 2.0f, false);
    Rect ground(Vec2(0.0f, 8.0f), Vec2(14.0f, 1.0f), 0.0f, 0.0f, true);

    addBody(&first);
    addBody(&second);
    addBody(&ground);

    const float frameDt = 1.0f / 30.0f;
    const int frameCount = 9999;
    std::cout << "frame, first (x, y, angle), second (x, y, angle)\n";

    for (int frame = 0; frame <= frameCount; ++frame) {
        if (frame % 60 == 0) {
            std::cout << frame << ", (" << first.pos.x << ", " << first.pos.y
                      << ", " << first.angle << "), (" << second.pos.x << ", "
                      << second.pos.y << ", " << second.angle << ")\n";
        }

        if (frame < frameCount)
            simulate(frameDt);
    }

    return 0;
}

int main() {
    Simulation simulation;
    
    simulation.runDemo();

    std::cin.get();
    
    return 0;

}
