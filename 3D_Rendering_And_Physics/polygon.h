#pragma once

#include "mylinal.h"

//Polygon
struct Polygon {
    Vec3 r1, r2, r3;
    int r, g, b;

    Polygon() : r1(0), r2(0), r3(0), r(255), g(255), b(255) {}
    Polygon(Vec3 r1, Vec3 r2, Vec3 r3, int r = 255, int g = 255, int b = 255) : r1(r1), r2(r2), r3(r3), r(r), g(g), b(b) {}
    Polygon(float x1, float y1, float z1,
        float x2, float y2, float z2,
        float x3, float y3, float z3,
        int r = 255, int g = 255, int b = 255) :
        r1(x1, y1, z1), r2(x2, y2, z2), r3(x3, y3, z3), r(r), g(g), b(b) {}

    Polygon rotAround(const Mat3x3& orientMat, const Vec3& originPoint = Vec3()) {
        Polygon pRot(*this);

        pRot.r1 = orientMat * (r1 - originPoint) + originPoint;
        pRot.r2 = orientMat * (r2 - originPoint) + originPoint;
        pRot.r3 = orientMat * (r3 - originPoint) + originPoint;

        return pRot;
    }
};
inline Polygon operator+(const Polygon& poly, const Vec3& vec) {
    Polygon newPoly(poly);
    newPoly.r1 = poly.r1 + vec;
    newPoly.r2 = poly.r2 + vec;
    newPoly.r3 = poly.r3 + vec;
    return newPoly;
}
inline Polygon operator*(const Mat3x3& mat, const Polygon& poly) {
    Polygon newPoly(poly);
    newPoly.r1 = mat * poly.r1;
    newPoly.r2 = mat * poly.r2;
    newPoly.r3 = mat * poly.r3;
    return newPoly;
}
inline void makeRightHand(Polygon* poly) {
    if (tripleProd(poly->r1, poly->r2, poly->r3) > 0.f) return;
    Vec3 tmp = poly->r3;
    poly->r3 = poly->r2;
    poly->r2 = tmp;
}
inline float findSignedTetraVolume(Polygon& poly) {
    return tripleProd(poly.r1, poly.r2, poly.r3) / 6.f;
}
inline Vec3 findTetraCM(Polygon& poly) {
    return (poly.r1 + poly.r2 + poly.r3) / 4.f;
}
Mat3x3 findSignedTetraInertTen(Polygon& poly) {
    float Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    float& x2 = poly.r1.x; float& y2 = poly.r1.y; float& z2 = poly.r1.z;
    float& x3 = poly.r2.x; float& y3 = poly.r2.y; float& z3 = poly.r2.z;
    float& x4 = poly.r3.x; float& y4 = poly.r3.y; float& z4 = poly.r3.z;

    Ixx = (y2 * y2 + y2 * y3 + y3 * y3 + y2 * y4 + y3 * y4 + y4 * y4 + z2 * z2 + z2 * z3 + z3 * z3 + z2 * z4 + z3 * z4 + z4 * z4) / 60.f;
    Iyy = (x2 * x2 + x2 * x3 + x3 * x3 + x2 * x4 + x3 * x4 + x4 * x4 + z2 * z2 + z2 * z3 + z3 * z3 + z2 * z4 + z3 * z4 + z4 * z4) / 60.f;
    Izz = (x2 * x2 + x2 * x3 + x3 * x3 + x2 * x4 + x3 * x4 + x4 * x4 + y2 * y2 + y2 * y3 + y3 * y3 + y2 * y4 + y3 * y4 + y4 * y4) / 60.f;

    Ixy = (2 * x2 * y2 + x3 * y2 + x4 * y2 + x2 * y3 + 2 * x3 * y3 + x4 * y3 + x2 * y4 + x3 * y4 + 2 * x4 * y4) / 120.f;
    Ixz = (2 * x2 * z2 + x3 * z2 + x4 * z2 + x2 * z3 + 2 * x3 * z3 + x4 * z3 + x2 * z4 + x3 * z4 + 2 * x4 * z4) / 120.f;
    Iyz = (2 * y2 * z2 + y3 * z2 + y4 * z2 + y2 * z3 + 2 * y3 * z3 + y4 * z3 + y2 * z4 + y3 * z4 + 2 * y4 * z4) / 120.f;

    float jacobianDet = tripleProd(poly.r1, poly.r2, poly.r3);

    return jacobianDet * Mat3x3(Ixx, -Ixy, -Ixz,
        -Ixy, Iyy, -Iyz,
        -Ixz, -Iyz, Izz);
}

const Polygon polyOX(0, 0, 0, 300, 0, 0, 0, 10, 0, 255, 0, 0);
const Polygon polyOY(0, 0, 0, 0, 300, 0, 0, 0, 10, 0, 255, 0);
const Polygon polyOZ(0, 0, 0, 0, 0, 300, 10, 0, 0, 0, 0, 255);