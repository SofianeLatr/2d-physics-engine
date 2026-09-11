#ifndef CONTACT_POINTS_HPP
#define CONTACT_POINTS_HPP

#include <array>

void getContactPoints(const Body& A, const Body& B, std::vector<Vec2>& contactPoints);
#endif