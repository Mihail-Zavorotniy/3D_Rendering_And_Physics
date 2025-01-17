#pragma once

#include "mylinal.h"

//Light source
struct LightSource {
    Vec3 r;
    float rad;

    LightSource(float x, float y, float z, float rad) : r(Vec3(x, y, z)), rad(rad) {}

    void rotAround(const Vec3& axisVec, const float angle, const Vec3& originPoint = Vec3()) {
        r = createRotMat(axisVec, angle) * (r - originPoint) + originPoint;
    }
};