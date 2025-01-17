#pragma once

#include <iostream>
#include <cmath>

using namespace std;

//Prototypes
struct Vec3;
struct Mat3x3;
float modSqr(const Vec3&);
float mod(const Vec3&);
Vec3 normalize(const Vec3&);
float dotProd(const Vec3&, const Vec3&);
Vec3 crossProd(const Vec3&, const Vec3&);
float det(const Mat3x3&);
Mat3x3 createRotMat(const Vec3&, float);
Vec3 operator*(const Mat3x3&, const Vec3&);

//Vector
struct Vec3 {
    float x, y, z;

    explicit Vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}

    Vec3& operator=(const Vec3& other) {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }
    Vec3 operator-() const {
        Vec3 v;
        v.x = -x;
        v.y = -y;
        v.z = -z;
        return v;
    }
    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
    Vec3 operator+(const Vec3& other) const {
        Vec3 buff(*this);
        buff += other;
        return buff;
    }
    Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }
    Vec3 operator-(const Vec3& other) const {
        Vec3 buff(*this);
        buff -= other;
        return buff;
    }
    Vec3& operator*=(const float a) {
        x *= a;
        y *= a;
        z *= a;
        return *this;
    }
    Vec3 operator*(const float a) const {
        Vec3 buff(*this);
        buff *= a;
        return buff;
    }
    Vec3& operator/=(const float a) {
        x /= a;
        y /= a;
        z /= a;
        return *this;
    }
    Vec3 operator/(const float a) const {
        Vec3 buff(*this);
        buff /= a;
        return buff;
    }
    Vec3 projOn(const Vec3& other) const {
        return other * dotProd(*this, other) / dotProd(other, other);
    }
};
Vec3 operator*(const float a, const Vec3& vec) {
    return vec * a;
}
ostream& operator<<(ostream& os, const Vec3& vec) {
    os << vec.x << " " << vec.y << " " << vec.z;
    return os;
}
float modSqr(const Vec3& vec) { //Returns squared module of a vector
    return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
}
float mod(const Vec3& vec) { //Returns module of a vector
    return sqrt(modSqr(vec));
}
Vec3 normalize(const Vec3& vec) {
    return vec / mod(vec);
}
float dotProd(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
float normDotProd(const Vec3& a, const Vec3& b) {
    return dotProd(a, b) / sqrt(modSqr(a) * modSqr(b));
}
Vec3 crossProd(const Vec3& a, const Vec3& b) {
    return Vec3(a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x);
}
float tripleProd(const Vec3& a, const Vec3& b, const Vec3& c) {
    return a.x * (b.y * c.z - b.z * c.y) - a.y * (b.x * c.z - b.z * c.x) + a.z * (b.x * c.y - b.y * c.x);
}
Vec3 findIntersection(Vec3 line, Vec3 r1, Vec3 r2, Vec3 r3) {
    Vec3 n = crossProd(r2 - r1, r3 - r1);
    return line * dotProd(n, r1) / dotProd(n, line);
}
const Vec3 xUnit(1, 0, 0);
const Vec3 yUnit(0, 1, 0);
const Vec3 zUnit(0, 0, 1);

//Matrix
struct Mat3x3 {
    float a1, a2, a3,
        b1, b2, b3,
        c1, c2, c3;

    explicit Mat3x3(float a1 = 0, float a2 = 0, float a3 = 0, float b1 = 0, float b2 = 0, float b3 = 0, float c1 = 0, float c2 = 0, float c3 = 0) :
        a1(a1), a2(a2), a3(a3), b1(b1), b2(b2), b3(b3), c1(c1), c2(c2), c3(c3) {}

    Mat3x3& operator=(const Mat3x3& other) {
        a1 = other.a1; a2 = other.a2; a3 = other.a3;
        b1 = other.b1; b2 = other.b2; b3 = other.b3;
        c1 = other.c1; c2 = other.c2; c3 = other.c3;
        return *this;
    }
    Mat3x3& operator+=(const Mat3x3& other) {
        a1 += other.a1; a2 += other.a2; a3 += other.a3;
        b1 += other.b1; b2 += other.b2; b3 += other.b3;
        c1 += other.c1; c2 += other.c2; c3 += other.c3;
        return *this;
    }
    Mat3x3 operator+(const Mat3x3& other) const {
        Mat3x3 buff(*this);
        buff += other;
        return buff;
    }
    Mat3x3& operator-=(const Mat3x3& other) {
        a1 -= other.a1; a2 -= other.a2; a3 -= other.a3;
        b1 -= other.b1; b2 -= other.b2; b3 -= other.b3;
        c1 -= other.c1; c2 -= other.c2; c3 -= other.c3;
        return *this;
    }
    Mat3x3 operator-(const Mat3x3& other) const {
        Mat3x3 buff(*this);
        buff -= other;
        return buff;
    }
    Mat3x3& operator*=(const float a) {
        a1 *= a; a2 *= a; a3 *= a;
        b1 *= a; b2 *= a; b3 *= a;
        c1 *= a; c2 *= a; c3 *= a;
        return *this;
    }
    Mat3x3 operator*(const float a) const {
        Mat3x3 buff(*this);
        buff *= a;
        return buff;
    }
    Mat3x3& operator/=(const float a) {
        a1 /= a; a2 /= a; a3 /= a;
        b1 /= a; b2 /= a; b3 /= a;
        c1 /= a; c2 /= a; c3 /= a;
        return *this;
    }
    Mat3x3 operator/(const float a) const {
        Mat3x3 buff(*this);
        buff /= a;
        return buff;
    }
    Mat3x3 operator*(const Mat3x3& mat) const {
        return Mat3x3(a1 * mat.a1 + a2 * mat.b1 + a3 * mat.c1, a1 * mat.a2 + a2 * mat.b2 + a3 * mat.c2, a1 * mat.a3 + a2 * mat.b3 + a3 * mat.c3,
            b1 * mat.a1 + b2 * mat.b1 + b3 * mat.c1, b1 * mat.a2 + b2 * mat.b2 + b3 * mat.c2, b1 * mat.a3 + b2 * mat.b3 + b3 * mat.c3,
            c1 * mat.a1 + c2 * mat.b1 + c3 * mat.c1, c1 * mat.a2 + c2 * mat.b2 + c3 * mat.c2, c1 * mat.a3 + c2 * mat.b3 + c3 * mat.c3);
    }
    Mat3x3 T() const {
        return Mat3x3(a1, b1, c1,
            a2, b2, c2,
            a3, b3, c3);
    }
    Mat3x3 inv() const {
        Mat3x3 invMat(0);
        float invDet = 1.f / det(*this);

        invMat.a1 = (b2 * c3 - b3 * c2) * invDet; invMat.a2 = (c2 * a3 - a2 * c3) * invDet; invMat.a3 = (a2 * b3 - a3 * b2) * invDet;
        invMat.b1 = (c1 * b3 - b1 * c3) * invDet; invMat.b2 = (a1 * c3 - c1 * a3) * invDet; invMat.b3 = (a3 * b1 - a1 * b3) * invDet;
        invMat.c1 = (b1 * c2 - c1 * b2) * invDet; invMat.c2 = (a2 * c1 - a1 * c2) * invDet; invMat.c3 = (a1 * b2 - a2 * b1) * invDet;

        return invMat;
    }
};
Mat3x3 operator*(const float a, const Mat3x3& mat) {
    return mat * a;
}
Vec3 operator*(const Mat3x3& mat, const Vec3& vec) {
    return Vec3(mat.a1 * vec.x + mat.a2 * vec.y + mat.a3 * vec.z,
        mat.b1 * vec.x + mat.b2 * vec.y + mat.b3 * vec.z,
        mat.c1 * vec.x + mat.c2 * vec.y + mat.c3 * vec.z);
}
ostream& operator<<(ostream& os, const Mat3x3& mat) {
    os << mat.a1 << " " << mat.a2 << " " << mat.a3 << "\n";
    os << mat.b1 << " " << mat.b2 << " " << mat.b3 << "\n";
    os << mat.c1 << " " << mat.c2 << " " << mat.c3 << "\n";
    return os;
}
float det(const Mat3x3& mat) {
    return mat.a1 * (mat.b2 * mat.c3 - mat.b3 * mat.c2)
        - mat.a2 * (mat.b1 * mat.c3 - mat.b3 * mat.c1)
        + mat.a3 * (mat.b1 * mat.c2 - mat.b2 * mat.c1);
}
Mat3x3 createRotMat(const Vec3& axisVec, float angle) {
    Mat3x3 orientMat;

    float cosT(cos(angle)), sinT(sin(angle)), oneMinCos(1 - cosT);
    Vec3 axisUnitVec = normalize(axisVec);
    float& x = axisUnitVec.x;
    float& y = axisUnitVec.y;
    float& z = axisUnitVec.z;

    orientMat.a1 = x * x * oneMinCos + cosT;   orientMat.a2 = x * y * oneMinCos - z * sinT;   orientMat.a3 = x * z * oneMinCos + y * sinT;
    orientMat.b1 = x * y * oneMinCos + z * sinT;   orientMat.b2 = y * y * oneMinCos + cosT;   orientMat.b3 = y * z * oneMinCos - x * sinT;
    orientMat.c1 = x * z * oneMinCos - y * sinT;   orientMat.c2 = y * z * oneMinCos + x * sinT;   orientMat.c3 = z * z * oneMinCos + cosT;

    return orientMat;
}
Mat3x3 TensorFromAnyToCM(const Mat3x3& tensor, const Vec3& a, float mass) {
    Mat3x3 mat{ a.y * a.y + a.z * a.z,         -a.x * a.y,              -a.x * a.z,
                     -a.x * a.y,          a.x * a.x + a.z * a.z,        -a.y * a.z,
                     -a.x * a.z,               -a.y * a.z,         a.x * a.x + a.y * a.y };

    return tensor - mass * mat;
}
Mat3x3 TensorFromCMToAny(const Mat3x3& tensor, const Vec3& a, float mass) {
    Mat3x3 mat{ a.y * a.y + a.z * a.z,         -a.x * a.y,              -a.x * a.z,
                     -a.x * a.y,          a.x * a.x + a.z * a.z,        -a.y * a.z,
                     -a.x * a.z,               -a.y * a.z,         a.x * a.x + a.y * a.y };

    return tensor + mass * mat;
}
const Mat3x3 IdMat(1, 0, 0, 0, 1, 0, 0, 0, 1);