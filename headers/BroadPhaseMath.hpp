#ifndef BroadPhaseMath_HPP
#define BroadPhaseMath_HPP

#include "Body.hpp"
#include <vector>

struct AABB {
    Vec2 min,max;
};

struct AABBNode {
    AABB box;

    AABBNode* parent = nullptr;
    AABBNode* left = nullptr;
    AABBNode* right = nullptr;

    Body* body = nullptr;

    bool isLeaf() const {
        return left == nullptr;
    }
};

class AABBTree {
private:
    AABBNode* root = nullptr;

public:
    ~AABBTree();

    AABBNode* insert(Body* body,const AABB& box);
    void remove(AABBNode* node);
    void update(AABBNode* node,const AABB& box);
    void query(const AABB& box,std::vector<Body*>& bodies);

private:
    void insertLeaf(AABBNode* node);
    void removeLeaf(AABBNode* node);
    void destroy(AABBNode* node);

    AABB combine(const AABB& a,const AABB& b);
    float perimeter(const AABB& box);
};

#endif