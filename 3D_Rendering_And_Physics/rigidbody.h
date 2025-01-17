#pragma once

#include <vector>
#include "polygon.h"

//Rigid body
struct RigidBody {
    int polyNum = 0;
    vector<Polygon> polys;

    float volume = 0.f;
    float mass = 0.f;

    Vec3 cmPos = Vec3();
    Vec3 cmVel = Vec3();
    Vec3 angVel = Vec3();
    Vec3 angMom = Vec3();

    Mat3x3 invInertiaTensor = Mat3x3();
    Mat3x3 orientMat = IdMat;


    RigidBody() {}

    void addPoly(Vec3 r1, Vec3 r2, Vec3 r3, int r = 255, int g = 255, int b = 255) {
        polys.push_back(Polygon(r1, r2, r3, r, g, b));
        polyNum++;
    }
    void colorPoly(int id, int r, int g, int b) {
        polys[id].r = r;
        polys[id].g = g;
        polys[id].b = b;
    }
    void scale(float k) {
        for (int i = 0; i != polyNum; i++) {
            polys[i].r1 *= k;
            polys[i].r2 *= k;
            polys[i].r3 *= k;
        }
    }

    void bodyMove(const Vec3& displVec) {
        cmPos += displVec;
    }
    void bodyRotAround(const Mat3x3& rotMat, const Vec3& originPoint = Vec3()) {
        orientMat = rotMat * orientMat;
    }

    void integrator(float dt) {
        angVel = orientMat * invInertiaTensor * orientMat.T() * angMom;

        bodyMove(cmVel * dt);
        bodyRotAround(createRotMat(angVel, mod(angVel) * dt), cmPos);
    }
};
RigidBody glueTogether(const RigidBody& b1, const RigidBody& b2) {
    RigidBody newBody;

    newBody.mass = b1.mass + b2.mass;
    newBody.volume = b1.volume + b2.volume;

    newBody.cmPos = (b1.cmPos * b1.mass + b2.cmPos * b2.mass) / newBody.mass;
    newBody.cmVel = Vec3();
    newBody.angVel = Vec3();
    newBody.angMom = Vec3();

    newBody.polyNum = b1.polyNum + b2.polyNum;
    Vec3 displVec1 = b1.cmPos - newBody.cmPos;
    for (int i = 0; i != b1.polyNum; i++) {
        newBody.polys.push_back(b1.orientMat * b1.polys[i] + displVec1);
    }
    Vec3 displVec2 = b2.cmPos - newBody.cmPos;
    for (int i = 0; i != b2.polyNum; i++) {
        newBody.polys.push_back(b2.orientMat * b2.polys[i] + displVec2);
    }

    Mat3x3 newInertiaTensor = TensorFromCMToAny(b1.orientMat * b1.invInertiaTensor.inv() * b1.orientMat.T(), newBody.cmPos - b1.cmPos, b1.mass) +
        TensorFromCMToAny(b2.orientMat * b2.invInertiaTensor.inv() * b2.orientMat.T(), newBody.cmPos - b2.cmPos, b2.mass);
    newBody.invInertiaTensor = newInertiaTensor.inv();
    newBody.orientMat = IdMat;

    return newBody;
}
RigidBody createBodyFromMesh(float density, vector<Polygon> polys) {
    RigidBody body;

    body.polyNum = polys.size();
    body.polys = polys;

    Mat3x3 bodyInertTen = Mat3x3();

    for (int i = 0; i != body.polyNum; i++) {
        Polygon poly = polys[i];
        float vol = findSignedTetraVolume(poly);

        body.volume += vol;
        body.cmPos += vol * findTetraCM(poly);
        bodyInertTen += findSignedTetraInertTen(poly);
    }

    body.mass = density * body.volume;
    body.cmPos = body.cmPos / body.volume; //can divide by volume instead of mass because density is constant
    bodyInertTen = TensorFromAnyToCM(density * bodyInertTen, body.cmPos, body.mass);
    body.invInertiaTensor = bodyInertTen.inv();

    return body;
}

RigidBody createCuboid(float dens, float x, float y, float z) {
    RigidBody cuboid;

    Vec3 v1(x / 2, y / 2, z / 2);
    Vec3 v2(x / 2, y / 2, -z / 2);
    Vec3 v3(x / 2, -y / 2, z / 2);
    Vec3 v4(x / 2, -y / 2, -z / 2);
    Vec3 v5(-x / 2, y / 2, z / 2);
    Vec3 v6(-x / 2, y / 2, -z / 2);
    Vec3 v7(-x / 2, -y / 2, z / 2);
    Vec3 v8(-x / 2, -y / 2, -z / 2);

    cuboid.addPoly(v1, v2, v3);
    cuboid.addPoly(v4, v2, v3);
    cuboid.addPoly(v5, v6, v7);
    cuboid.addPoly(v8, v6, v7);
    cuboid.addPoly(v1, v3, v5);
    cuboid.addPoly(v7, v3, v5);
    cuboid.addPoly(v2, v4, v6);
    cuboid.addPoly(v8, v4, v6);
    cuboid.addPoly(v1, v2, v5);
    cuboid.addPoly(v6, v2, v5);
    cuboid.addPoly(v3, v4, v7);
    cuboid.addPoly(v8, v4, v7);

    for (int i = 0; i != cuboid.polyNum; i++) {
        makeRightHand(&cuboid.polys[i]);
    }

    cuboid.volume = x * y * z;
    cuboid.mass = dens * cuboid.volume;

    cuboid.cmPos = Vec3();
    cuboid.cmVel = Vec3();
    cuboid.angVel = Vec3();
    cuboid.angMom = Vec3();

    cuboid.orientMat = IdMat;

    float& m = cuboid.mass;
    cuboid.invInertiaTensor = Mat3x3();
    cuboid.invInertiaTensor.a1 = 12.f / (m * (y * y + z * z));
    cuboid.invInertiaTensor.b2 = 12.f / (m * (x * x + z * z));
    cuboid.invInertiaTensor.c3 = 12.f / (m * (x * x + y * y));

    return cuboid;
}
RigidBody createIcosahedron(float dens, float icosR) {
    vector<Polygon> polygons;
    float phi = 0.5f * (1 + sqrt(5));

    Vec3 v1(phi * icosR, icosR, 0);
    Vec3 v2(phi * icosR, -icosR, 0);
    Vec3 v3(-phi * icosR, -icosR, 0);
    Vec3 v4(-phi * icosR, icosR, 0);
    Vec3 v5(icosR, 0, phi * icosR);
    Vec3 v6(-icosR, 0, phi * icosR);
    Vec3 v7(-icosR, 0, -phi * icosR);
    Vec3 v8(icosR, 0, -phi * icosR);
    Vec3 v9(0, phi * icosR, icosR);
    Vec3 v10(0, phi * icosR, -icosR);
    Vec3 v11(0, -phi * icosR, -icosR);
    Vec3 v12(0, -phi * icosR, icosR);

    polygons.push_back(Polygon(v5, v6, v9));
    polygons.push_back(Polygon(v5, v6, v12));
    polygons.push_back(Polygon(v5, v9, v1));
    polygons.push_back(Polygon(v6, v9, v4));
    polygons.push_back(Polygon(v6, v12, v3));
    polygons.push_back(Polygon(v5, v12, v2));
    polygons.push_back(Polygon(v5, v1, v2));
    polygons.push_back(Polygon(v6, v4, v3));
    polygons.push_back(Polygon(v3, v12, v11));
    polygons.push_back(Polygon(v2, v12, v11));
    polygons.push_back(Polygon(v1, v9, v10));
    polygons.push_back(Polygon(v4, v9, v10));
    polygons.push_back(Polygon(v1, v2, v8));
    polygons.push_back(Polygon(v3, v4, v7));
    polygons.push_back(Polygon(v1, v8, v10));
    polygons.push_back(Polygon(v2, v8, v11));
    polygons.push_back(Polygon(v3, v7, v11));
    polygons.push_back(Polygon(v4, v7, v10));
    polygons.push_back(Polygon(v7, v8, v10));
    polygons.push_back(Polygon(v7, v8, v11));

    for (int i = 0; i != 20; i++) {
        makeRightHand(&polygons[i]);
    }

    RigidBody icosahedron = createBodyFromMesh(dens, polygons);

    return icosahedron;
}
//RigidBody createHammer(float dens, )