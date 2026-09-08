#include "Math.hpp"
#include <algorithm>

AABBTree::~AABBTree()
{
    destroy(root);
}

AABBTree::AABBNode* AABBTree::insert(Body* body,const AABB& box)
{
    AABBNode* node = new AABBNode;

    node->box = box;
    node->body = body;

    insertLeaf(node);

    return node;
}

void AABBTree::remove(AABBNode* node)
{
    removeLeaf(node);
    delete node;
}

void AABBTree::update(AABBNode* node,const AABB& box)
{
    if(node->box.min.x <= box.min.x &&
       node->box.min.y <= box.min.y &&
       node->box.max.x >= box.max.x &&
       node->box.max.y >= box.max.y)
        return;

    Body* body = node->body;

    remove(node);
    insert(body,box);
}

void AABBTree::query(const AABB& box,std::vector<Body*>& bodies)
{
    if(root == nullptr)
        return;

    std::vector<AABBNode*> stack;
    stack.push_back(root);

    while(!stack.empty())
    {
        AABBNode* node = stack.back();
        stack.pop_back();

        if(node->box.max.x < box.min.x ||
           node->box.min.x > box.max.x ||
           node->box.max.y < box.min.y ||
           node->box.min.y > box.max.y)
            continue;

        if(node->isLeaf())
            bodies.push_back(node->body);
        else
        {
            stack.push_back(node->left);
            stack.push_back(node->right);
        }
    }
}

void AABBTree::insertLeaf(AABBNode* node)
{
    if(root == nullptr)
    {
        root = node;
        return;
    }

    AABBNode* current = root;

    while(!current->isLeaf())
    {
        AABBNode* left = current->left;
        AABBNode* right = current->right;

        AABB combinedLeft = combine(left->box,node->box);
        AABB combinedRight = combine(right->box,node->box);

        float leftCost = perimeter(combinedLeft) - perimeter(left->box);
        float rightCost = perimeter(combinedRight) - perimeter(right->box);

        current = leftCost < rightCost ? left : right;
    }

    AABBNode* oldParent = current->parent;

    AABBNode* newParent = new AABBNode;

    newParent->parent = oldParent;
    newParent->left = current;
    newParent->right = node;
    newParent->box = combine(current->box,node->box);

    current->parent = newParent;
    node->parent = newParent;

    if(oldParent == nullptr)
        root = newParent;
    else if(oldParent->left == current)
        oldParent->left = newParent;
    else
        oldParent->right = newParent;

    current = newParent->parent;

    while(current)
    {
        current->box = combine(current->left->box,current->right->box);
        current = current->parent;
    }
}

void AABBTree::removeLeaf(AABBNode* node)
{
    if(node == root)
    {
        root = nullptr;
        return;
    }

    AABBNode* parent = node->parent;
    AABBNode* grandParent = parent->parent;

    AABBNode* sibling;

    if(parent->left == node)
        sibling = parent->right;
    else
        sibling = parent->left;

    if(grandParent == nullptr)
    {
        root = sibling;
        sibling->parent = nullptr;
    }
    else
    {
        if(grandParent->left == parent)
            grandParent->left = sibling;
        else
            grandParent->right = sibling;

        sibling->parent = grandParent;

        AABBNode* current = grandParent;

        while(current)
        {
            current->box = combine(current->left->box,current->right->box);
            current = current->parent;
        }
    }

    delete parent;
}

void AABBTree::destroy(AABBNode* node)
{
    if(node == nullptr)
        return;

    destroy(node->left);
    destroy(node->right);

    delete node;
}

AABB AABBTree::combine(const AABB& a,const AABB& b)
{
    AABB result;

    result.min.x = std::min(a.min.x,b.min.x);
    result.min.y = std::min(a.min.y,b.min.y);

    result.max.x = std::max(a.max.x,b.max.x);
    result.max.y = std::max(a.max.y,b.max.y);

    return result;
}

float AABBTree::perimeter(const AABB& box)
{
    float width = box.max.x - box.min.x;
    float height = box.max.y - box.min.y;

    return 2.0f * (width + height);
}