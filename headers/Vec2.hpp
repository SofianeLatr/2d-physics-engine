#ifndef Vec2_h
#define Vec2_h
#include <cmath>
class Vec2 {
public:
    float x,y;
    ~Vec2();
    Vec2();
    Vec2(float x,float y);
    float operator *(const Vec2& other) const { return x*other.x+y*other.y; }
    Vec2 operator +(const Vec2& other) const { return Vec2(x+other.x,y+other.y); }
    Vec2 operator -(const Vec2& other) const { return Vec2(x-other.x,y-other.y); }
    void operator +=(Vec2 other) { x+=other.x;y+=other.y; }
    void operator -=(Vec2 other) { x-=other.x;y-=other.y; }
    void operator *=(float other) { x*=other;y*=other; }
    Vec2& operator = (Vec2 other) { x=other.x;y=other.y;return *this; }
    Vec2 normal() const { return Vec2(-y,x); }
    float cross(Vec2 other) { return x*other.y-y*other.x; }
    float distance(Vec2 other) {return std::sqrt((x-other.x)*(x-other.x)+(y-other.y)*(y-other.y));}
    float length() {return std::sqrt(x*x+y*y);}
    // Normalize this vector in place. A zero vector remains unchanged.
    Vec2& normalize();
};

inline Vec2 operator *(float a,Vec2 b) { return Vec2(a*b.x,a*b.y); }
inline Vec2 operator *(Vec2 b,float a) { return Vec2(a*b.x,a*b.y); }
inline Vec2 operator /(Vec2 b,float a) { return Vec2(b.x/a,b.y/a); }

#endif
