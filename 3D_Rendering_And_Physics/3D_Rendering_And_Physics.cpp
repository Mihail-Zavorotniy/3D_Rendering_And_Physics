#include <SDL.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <chrono>
#include <vector>
#include <limits>
#include <algorithm>
using namespace std;

//Parameters
const int   FPS = 30;
const int   WIDTH = 800;
const int   HEIGHT = 600;
const int   WINDOW_WIDTH = WIDTH;
const int   WINDOW_HEIGHT = HEIGHT;
const float TIMESTEP = 0.01f;
const float FOV = 90.f;
const float CAM_INIT_X = 0.f;
const float CAM_INIT_Y = -400.f;
const float CAM_INIT_Z = 0.f;
const float CAM_LIN_SPEED = 6.f;
const float CAM_ROT_SPEED = 0.002f;
const float GLOSS_FACTOR = 200;

float zBuff[HEIGHT][WIDTH] = { numeric_limits<float>::max() };
Uint32 preLightBuff[HEIGHT][WIDTH][3] = { 0 };
Uint32 postLightBuff[HEIGHT * WIDTH] = { 0 };

//Inner variables
const long long tickCap = int(1000 / FPS);
long long start, tickTime;
SDL_Event event;
int tickCnt = 0;
bool quit = false;

//Input variables
bool wKey(false), aKey(false), sKey(false), dKey(false), altKey(false), spaceKey(false), qKey(false), eKey(false), mouseButton(false), mouseMotion(false);
float mdx(0), mdy(0);

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

//Functions
bool pointInTriangle(float px, float py, float x1, float y1, float x2, float y2, float x3, float y3) {
    if ((x2 - x1) * (y3 - y1) > (y2 - y1) * (x3 - x1)) {
        if ((x2 - x1) * (py - y1) < (y2 - y1) * (px - x1)) return false;
        if ((x3 - x2) * (py - y2) < (y3 - y2) * (px - x2)) return false;
        if ((x1 - x3) * (py - y3) < (y1 - y3) * (px - x3)) return false;
    }
    else {
        if ((x2 - x1) * (py - y1) > (y2 - y1) * (px - x1)) return false;
        if ((x3 - x2) * (py - y2) > (y3 - y2) * (px - x2)) return false;
        if ((x1 - x3) * (py - y3) > (y1 - y3) * (px - x3)) return false;
    }
    return true;
}
inline Uint32 rgbToHex(int r, int g, int b) {
    return (r << 16) + (g << 8) + b;
}
inline int hexToRed(Uint32 hex) {
    return ((hex & 0xff0000) >> 16);
}
inline int hexToGreen(Uint32 hex) {
    return ((hex & 0x00ff00) >> 8);
}
inline int hexToBlue(Uint32 hex) {
    return (hex & 0x0000ff);
}

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
Vec3 xUnit(1, 0, 0);
Vec3 yUnit(0, 1, 0);
Vec3 zUnit(0, 0, 1);

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
inline Mat3x3 IdMat() {
    return Mat3x3(1, 0, 0,
        0, 1, 0,
        0, 0, 1);
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

//Light source
struct LightSource {
    Vec3 r;
    float rad;

    LightSource(float x, float y, float z, float rad) : r(Vec3(x, y, z)), rad(rad) {}

    void rotAround(const Vec3& axisVec, const float angle, const Vec3& originPoint = Vec3()) {
        r = createRotMat(axisVec, angle) * (r - originPoint) + originPoint;
    }
};

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
Vec3 directionBuff[HEIGHT][WIDTH];
Vec3 normalBuff[HEIGHT][WIDTH];

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
    Mat3x3 orientMat = IdMat();


    RigidBody() {}

    void addPoly(Vec3 r1, Vec3 r2, Vec3 r3, int r = 255, int g = 255, int b = 255) {
        polys.push_back(Polygon(r1, r2, r3, r, g, b));
        polyNum++;
    }

    inline void bodyMove(const Vec3& displVec) {
        cmPos += displVec;
    }
    inline void bodyRotAround(const Mat3x3& rotMat, const Vec3& originPoint = Vec3()) {
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
    newBody.orientMat = IdMat();

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
    body.cmPos = body.cmPos / body.volume;
    cout << density * bodyInertTen << "\n";
    bodyInertTen = TensorFromAnyToCM(density * bodyInertTen, body.cmPos, body.mass);
    cout << bodyInertTen << "\n";
    body.invInertiaTensor = bodyInertTen.inv();

    return body;
}
RigidBody createCuboid(float rho, float x, float y, float z) {
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
    cuboid.mass = rho * cuboid.volume;

    cuboid.cmPos = Vec3();
    cuboid.cmVel = Vec3();
    cuboid.angVel = Vec3();
    cuboid.angMom = Vec3();

    cuboid.orientMat = IdMat();

    float& m = cuboid.mass;
    cuboid.invInertiaTensor = Mat3x3();
    cuboid.invInertiaTensor.a1 = 12.f / (m * (y * y + z * z));
    cuboid.invInertiaTensor.b2 = 12.f / (m * (x * x + z * z));
    cuboid.invInertiaTensor.c3 = 12.f / (m * (x * x + y * y));

    return cuboid;
}
RigidBody createIcosahedron(float icosR) {
    RigidBody icosahedron;
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

    icosahedron.addPoly(v5, v6, v9);
    icosahedron.addPoly(v5, v6, v12);
    icosahedron.addPoly(v5, v9, v1);
    icosahedron.addPoly(v6, v9, v4);
    icosahedron.addPoly(v6, v12, v3);
    icosahedron.addPoly(v5, v12, v2);
    icosahedron.addPoly(v5, v1, v2);
    icosahedron.addPoly(v6, v4, v3);
    icosahedron.addPoly(v3, v12, v11);
    icosahedron.addPoly(v2, v12, v11);
    icosahedron.addPoly(v1, v9, v10);
    icosahedron.addPoly(v4, v9, v10);
    icosahedron.addPoly(v1, v2, v8);
    icosahedron.addPoly(v3, v4, v7);
    icosahedron.addPoly(v1, v8, v10);
    icosahedron.addPoly(v2, v8, v11);
    icosahedron.addPoly(v3, v7, v11);
    icosahedron.addPoly(v4, v7, v10);
    icosahedron.addPoly(v7, v8, v10);
    icosahedron.addPoly(v7, v8, v11);

    for (int i = 0; i != icosahedron.polyNum; i++) {
        makeRightHand(&icosahedron.polys[i]);
    }

    return icosahedron;
}

//Camera
struct Camera {
    SDL_Renderer* rend;
    Vec3 eye;
    float scale, pixelSize, width, height;
    const float planeDist = 100.f;
    Mat3x3 orientMat = IdMat();


    Camera(SDL_Renderer* rend, float x, float y, float z, float fov) : rend(rend), eye(x, y, z) {
        scale = WIDTH / (2.f * planeDist * tan(fov * M_PI / 720.f));
        pixelSize = 1.f / scale;
        width = WIDTH * pixelSize;
        height = HEIGHT * pixelSize;
    }

    inline Vec3 toCameraCS(const Vec3& vec) {
        return orientMat.T() * vec;
    }

    void updBuff(Vec3& r1, Vec3& r2, Vec3& r3, float x1, float y1, float x2, float y2, float x3, float y3, float red, float green, float blue) {

        int minX(max(0, int(0.5f * WIDTH + min({ x1, x2, x3 }) * scale)));
        int maxX(min(WIDTH, int(0.5f * WIDTH + max({ x1, x2, x3 }) * scale)));
        int minY(max(0, int(0.5f * HEIGHT + min({ y1, y2, y3 }) * scale)));
        int maxY(min(HEIGHT, int(0.5f * HEIGHT + max({ y1, y2, y3 }) * scale)));

        if (minX > WIDTH) return;
        if (maxX < 0) return;
        if (minY > HEIGHT) return;
        if (maxY < 0) return;

        float pxStart((minX - 0.5f * WIDTH) * pixelSize),
            px(pxStart),
            py((minY - 0.5f * HEIGHT) * pixelSize),
            polyDistSqr(0);
        Vec3 directionVec(0);

        for (int y = minY; y != maxY; y++) {
            for (int x = minX; x != maxX; x++) {
                if (!pointInTriangle(px, py, x1, y1, x2, y2, x3, y3)) {
                    px += pixelSize;
                    continue;
                }

                directionVec = findIntersection(Vec3(px, py, planeDist), r1, r2, r3);
                polyDistSqr = modSqr(directionVec);
                if (polyDistSqr < zBuff[y][x]) {
                    zBuff[y][x] = polyDistSqr;
                    directionBuff[y][x] = directionVec;
                    normalBuff[y][x] = crossProd(r2 - r1, r3 - r1);
                    preLightBuff[y][x][0] = red;
                    preLightBuff[y][x][1] = green;
                    preLightBuff[y][x][2] = blue;
                }

                px += pixelSize;
            }
            px = pxStart;
            py += pixelSize;
        }
    }
    void renderPolygon(Polygon poly) {
        Vec3 r1(toCameraCS(poly.r1 - eye)), r2(toCameraCS(poly.r2 - eye)), r3(toCameraCS(poly.r3 - eye));
        bool back1(false), back2(false), back3(false);

        if (r1.z < planeDist) back1 = true;
        if (r2.z < planeDist) back2 = true;
        if (r3.z < planeDist) back3 = true;


        if (!back1 and !back2 and !back3) {
            float x1, y1, x2, y2, x3, y3;

            x1 = r1.x * planeDist / r1.z;
            y1 = r1.y * planeDist / r1.z;

            x2 = r2.x * planeDist / r2.z;
            y2 = r2.y * planeDist / r2.z;

            x3 = r3.x * planeDist / r3.z;
            y3 = r3.y * planeDist / r3.z;

            updBuff(r1, r2, r3, x1, y1, x2, y2, x3, y3, poly.r, poly.g, poly.b);

            return;
        }
        else if (back1 and back2 and back3) {
            return;
        }
        else if (back1 and !back2 and !back3) {
            Vec3& f1 = r2; Vec3& f2 = r3; Vec3& b = r1;
            float x1, y1, x2, y2, x3, y3, x4, y4;

            x1 = f1.x * planeDist / f1.z;
            y1 = f1.y * planeDist / f1.z;

            x2 = f2.x * planeDist / f2.z;
            y2 = f2.y * planeDist / f2.z;

            x3 = b.x + (f1.x - b.x) * (planeDist - b.z) / (f1.z - b.z);
            y3 = b.y + (f1.y - b.y) * (planeDist - b.z) / (f1.z - b.z);

            x4 = b.x + (f2.x - b.x) * (planeDist - b.z) / (f2.z - b.z);
            y4 = b.y + (f2.y - b.y) * (planeDist - b.z) / (f2.z - b.z);

            updBuff(r1, r2, r3, x1, y1, x2, y2, x3, y3, poly.r, poly.g, poly.b);
            updBuff(r1, r2, r3, x2, y2, x3, y3, x4, y4, poly.r, poly.g, poly.b);

            return;
        }
        else if (!back1 and back2 and !back3) {
            Vec3& f1 = r1; Vec3& f2 = r3; Vec3& b = r2;
            float x1, y1, x2, y2, x3, y3, x4, y4;

            x1 = f1.x * planeDist / f1.z;
            y1 = f1.y * planeDist / f1.z;

            x2 = f2.x * planeDist / f2.z;
            y2 = f2.y * planeDist / f2.z;

            x3 = b.x + (f1.x - b.x) * (planeDist - b.z) / (f1.z - b.z);
            y3 = b.y + (f1.y - b.y) * (planeDist - b.z) / (f1.z - b.z);

            x4 = b.x + (f2.x - b.x) * (planeDist - b.z) / (f2.z - b.z);
            y4 = b.y + (f2.y - b.y) * (planeDist - b.z) / (f2.z - b.z);

            updBuff(r1, r2, r3, x1, y1, x2, y2, x3, y3, poly.r, poly.g, poly.b);
            updBuff(r1, r2, r3, x2, y2, x3, y3, x4, y4, poly.r, poly.g, poly.b);

            return;
        }
        else if (!back1 and !back2 and back3) {
            Vec3& f1 = r1; Vec3& f2 = r2; Vec3& b = r3;
            float x1, y1, x2, y2, x3, y3, x4, y4;

            x1 = f1.x * planeDist / f1.z;
            y1 = f1.y * planeDist / f1.z;

            x2 = f2.x * planeDist / f2.z;
            y2 = f2.y * planeDist / f2.z;

            x3 = b.x + (f1.x - b.x) * (planeDist - b.z) / (f1.z - b.z);
            y3 = b.y + (f1.y - b.y) * (planeDist - b.z) / (f1.z - b.z);

            x4 = b.x + (f2.x - b.x) * (planeDist - b.z) / (f2.z - b.z);
            y4 = b.y + (f2.y - b.y) * (planeDist - b.z) / (f2.z - b.z);

            updBuff(r1, r2, r3, x1, y1, x2, y2, x3, y3, poly.r, poly.g, poly.b);
            updBuff(r1, r2, r3, x2, y2, x3, y3, x4, y4, poly.r, poly.g, poly.b);

            return;
        }
        else if (back1 and back2 and !back3) {
            Vec3& f = r3; Vec3& b1 = r1; Vec3& b2 = r2;
            float x1, y1, x2, y2, x3, y3;

            x1 = f.x * planeDist / f.z;
            y1 = f.y * planeDist / f.z;

            x2 = b1.x + (f.x - b1.x) * (planeDist - b1.z) / (f.z - b1.z);
            y2 = b1.y + (f.y - b1.y) * (planeDist - b1.z) / (f.z - b1.z);

            x3 = b2.x + (f.x - b2.x) * (planeDist - b2.z) / (f.z - b2.z);
            y3 = b2.y + (f.y - b2.y) * (planeDist - b2.z) / (f.z - b2.z);

            updBuff(r1, r2, r3, x1, y1, x2, y2, x3, y3, poly.r, poly.g, poly.b);

            return;
        }
        else if (back1 and !back2 and back3) {
            Vec3& f = r2; Vec3& b1 = r1; Vec3& b2 = r3;
            float x1, y1, x2, y2, x3, y3;

            x1 = f.x * planeDist / f.z;
            y1 = f.y * planeDist / f.z;

            x2 = b1.x + (f.x - b1.x) * (planeDist - b1.z) / (f.z - b1.z);
            y2 = b1.y + (f.y - b1.y) * (planeDist - b1.z) / (f.z - b1.z);

            x3 = b2.x + (f.x - b2.x) * (planeDist - b2.z) / (f.z - b2.z);
            y3 = b2.y + (f.y - b2.y) * (planeDist - b2.z) / (f.z - b2.z);

            updBuff(r1, r2, r3, x1, y1, x2, y2, x3, y3, poly.r, poly.g, poly.b);

            return;
        }
        else if (!back1 and back2 and back3) {
            Vec3& f = r1; Vec3& b1 = r2; Vec3& b2 = r3;
            float x1, y1, x2, y2, x3, y3;

            x1 = f.x * planeDist / f.z;
            y1 = f.y * planeDist / f.z;

            x2 = b1.x + (f.x - b1.x) * (planeDist - b1.z) / (f.z - b1.z);
            y2 = b1.y + (f.y - b1.y) * (planeDist - b1.z) / (f.z - b1.z);

            x3 = b2.x + (f.x - b2.x) * (planeDist - b2.z) / (f.z - b2.z);
            y3 = b2.y + (f.y - b2.y) * (planeDist - b2.z) / (f.z - b2.z);

            updBuff(r1, r2, r3, x1, y1, x2, y2, x3, y3, poly.r, poly.g, poly.b);

            return;
        }
    }
    void renderShape(RigidBody& body) {
        Mat3x3 rotMat = body.orientMat;
        Vec3 displVec = body.cmPos;
        for (int i = 0; i != body.polys.size(); i++) {
            renderPolygon(rotMat * body.polys[i] + displVec);
        }
    }
    void applyLight(vector<LightSource> lights) {
        Vec3 directionVec(0), normalVec(0), incidentVec(0), reflectVec(0);
        float illumSum(0), glossSum(0), gloss(0);
        int red(0), green(0), blue(0);

        for (int y = 0; y != HEIGHT; y++) {
            for (int x = 0; x != WIDTH; x++) {
                red = preLightBuff[y][x][0];
                green = preLightBuff[y][x][1];
                blue = preLightBuff[y][x][2];

                if (red + green + blue == 0) continue;

                directionVec = directionBuff[y][x]; //already in eye CS
                normalVec = normalBuff[y][x];       //already in eye CS
                if (dotProd(directionVec, normalVec) > 0) normalVec = -1.f * normalVec;

                illumSum = 0;
                glossSum = 0;
                for (int i = 0; i != lights.size(); i++) {
                    incidentVec = toCameraCS(lights[i].r - eye) - directionVec;
                    illumSum += 0.5f * (normDotProd(normalVec, incidentVec) + 1.f) * lights[i].rad / modSqr(incidentVec);

                    reflectVec = incidentVec - 2.f * incidentVec.projOn(normalVec);
                    gloss = max(normDotProd(directionVec, reflectVec), 0.f);
                    gloss *= gloss;
                    gloss *= gloss;
                    glossSum += gloss * gloss;

                }

                illumSum = min(illumSum, 1.f);
                glossSum *= GLOSS_FACTOR;

                red = min(red * illumSum + glossSum, 255.f);
                green = min(green * illumSum + glossSum, 255.f);
                blue = min(blue * illumSum + glossSum, 255.f);

                postLightBuff[x + y * WIDTH] = rgbToHex(red, green, blue);
            }
        }
    }
    void draw(SDL_Texture* texture) {
        SDL_UpdateTexture(texture, NULL, postLightBuff, WIDTH * sizeof(Uint32));
        SDL_RenderCopy(rend, texture, NULL, NULL);

        for (int y = 0; y != HEIGHT; y++) {
            for (int x = 0; x != WIDTH; x++) {
                zBuff[y][x] = numeric_limits<float>::max();
                directionBuff[y][x] = Vec3();
                normalBuff[y][x] = Vec3();
                preLightBuff[y][x][0] = 0;
                preLightBuff[y][x][1] = 0;
                preLightBuff[y][x][2] = 0;
                postLightBuff[x + y * WIDTH] = 0;
            }
        }

    }

    void rotSelfOX(float angle) {
        orientMat = createRotMat(Vec3(orientMat.a1, orientMat.b1, orientMat.c1), angle) * orientMat;
    }
    void rotSelfOY(float angle) {
        orientMat = createRotMat(Vec3(orientMat.a2, orientMat.b2, orientMat.c2), angle) * orientMat;
    }
    void rotSelfOZ(float angle) {
        orientMat = createRotMat(Vec3(orientMat.a3, orientMat.b3, orientMat.c3), angle) * orientMat;
    }
    void rotWorldOX(float angle) {
        orientMat = createRotMat(xUnit, angle) * orientMat;
    }
    void rotWorldOY(float angle) {
        orientMat = createRotMat(yUnit, angle) * orientMat;
    }
    void rotWorldOZ(float angle) {
        orientMat = createRotMat(zUnit, angle) * orientMat;
    }

    void readKeyInput() {
        if (wKey)     eye += Vec3(orientMat.a3, orientMat.b3, orientMat.c3) * CAM_LIN_SPEED;
        if (sKey)     eye -= Vec3(orientMat.a3, orientMat.b3, orientMat.c3) * CAM_LIN_SPEED;
        if (dKey)     eye += Vec3(orientMat.a1, orientMat.b1, orientMat.c1) * CAM_LIN_SPEED;
        if (aKey)     eye -= Vec3(orientMat.a1, orientMat.b1, orientMat.c1) * CAM_LIN_SPEED;
        if (spaceKey) eye += zUnit * CAM_LIN_SPEED;
        if (altKey)   eye -= zUnit * CAM_LIN_SPEED;
        if (mouseMotion and mouseButton) {
            rotWorldOZ(-CAM_ROT_SPEED * mdx);
            rotSelfOX(-CAM_ROT_SPEED * mdy);
            mouseMotion = false; mdx = 0; mdy = 0;
        }
    }
};



//Main
int main(int argc, char* args[]) {
    //Creating window and renderer
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        return -1;
    }
    SDL_Window* window = SDL_CreateWindow("SDL Tutorial", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);
    if (window == nullptr) {
        printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
        return -1;
    }
    SDL_Renderer* rend = SDL_CreateRenderer(window, 0, SDL_RENDERER_ACCELERATED);
    SDL_Texture* texture = SDL_CreateTexture(rend, SDL_PIXELFORMAT_RGB888, SDL_TEXTUREACCESS_STREAMING, WINDOW_WIDTH, WINDOW_HEIGHT);

    //Objects
    Camera cam(rend, CAM_INIT_X, CAM_INIT_Y, CAM_INIT_Z, FOV);
    cam.rotSelfOX(-M_PI / 2);

    Polygon polyOX(0, 0, 0, 300, 0, 0, 0, 10, 0, 255, 0, 0);
    Polygon polyOY(0, 0, 0, 0, 300, 0, 0, 0, 10, 0, 255, 0);
    Polygon polyOZ(0, 0, 0, 0, 0, 300, 10, 0, 0, 0, 0, 255);
    RigidBody hammerHead = createCuboid(1e-4, 50, 100, 50);
    hammerHead.polys[4].g = 0;
    hammerHead.polys[4].b = 0;
    hammerHead.polys[5].g = 0;
    hammerHead.polys[5].b = 0;
    RigidBody hammerHandle = createCuboid(1e-4, 100, 10, 10);
    hammerHandle.bodyMove(Vec3(75, 0, 0));
    RigidBody hammer = glueTogether(hammerHead, hammerHandle);
    cout << hammer.orientMat * hammer.invInertiaTensor.inv() * hammer.orientMat.T();
    hammer.angMom = Vec3(5000, 0, 0.01);

    RigidBody icosahedron = createIcosahedron(50.f);

    vector<LightSource> lights;
    LightSource light1(0, 0, 300, 40000);
    lights.push_back(light1);

    auto t1 = chrono::system_clock::now().time_since_epoch();
    auto t2 = chrono::system_clock::now().time_since_epoch();

    //Main loop
    while (not quit) {
        //Write down starting time
        start = chrono::system_clock::now().time_since_epoch() / chrono::milliseconds(1);
        //Handle events
        while (SDL_PollEvent(&event)) {

            switch (event.type) {

            case SDL_MOUSEMOTION:
                mouseMotion = true;
                mdx = event.motion.xrel;
                mdy = event.motion.yrel;
                break;

            case SDL_MOUSEBUTTONDOWN:
                mouseButton = true;
                break;

            case SDL_MOUSEBUTTONUP:
                mouseButton = false;
                break;

            case SDL_KEYDOWN:

                switch (event.key.keysym.sym) {

                case SDLK_a:
                    aKey = true;
                    break;
                case SDLK_d:
                    dKey = true;
                    break;
                case SDLK_w:
                    wKey = true;
                    break;
                case SDLK_s:
                    sKey = true;
                    break;
                case SDLK_SPACE:
                    spaceKey = true;
                    break;
                case SDLK_LALT:
                    altKey = true;
                    break;
                case SDLK_q:
                    qKey = true;
                    break;
                case SDLK_e:
                    eKey = true;
                    break;
                default:
                    break;
                }
                break;

            case SDL_KEYUP:

                switch (event.key.keysym.sym) {

                case SDLK_a:
                    aKey = false;
                    break;
                case SDLK_d:
                    dKey = false;
                    break;
                case SDLK_w:
                    wKey = false;
                    break;
                case SDLK_s:
                    sKey = false;
                    break;
                case SDLK_SPACE:
                    spaceKey = false;
                    break;
                case SDLK_LALT:
                    altKey = false;
                    break;
                case SDLK_q:
                    qKey = false;
                    break;
                case SDLK_e:
                    eKey = false;
                    break;
                default:
                    break;
                }
                break;

            case SDL_QUIT:
                quit = true;
                break;

            default:
                break;
            }
        }

        //Drawing
        t1 = chrono::system_clock::now().time_since_epoch();

        //cam.renderShape(icosahedron);
        cam.renderPolygon(polyOX);
        cam.renderPolygon(polyOY);
        cam.renderPolygon(polyOZ);
        cam.renderShape(hammer);
        cam.applyLight(lights);

        t2 = chrono::system_clock::now().time_since_epoch();
        cout << (t2 - t1) / chrono::milliseconds(1) << "    ";


        t1 = chrono::system_clock::now().time_since_epoch();

        cam.draw(texture);

        t2 = chrono::system_clock::now().time_since_epoch();
        cout << (t2 - t1) / chrono::milliseconds(1) << "\n";


        SDL_RenderPresent(rend);

        //Camera movement
        cam.readKeyInput();

        //Objects movement
        for (int i = 0; i != 50; i++) {
            hammer.integrator(TIMESTEP);
        }

        //Sleep if neccessary
        tickTime = chrono::system_clock::now().time_since_epoch() / chrono::milliseconds(1) - start;
        if (tickTime < tickCap) {
            SDL_Delay(tickCap - tickTime);
        }
    }

    //Destroy window and renderer
    SDL_DestroyWindow(window);
    SDL_DestroyRenderer(rend);
    SDL_Quit();


    return 0;
}