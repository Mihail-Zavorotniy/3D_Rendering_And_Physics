#pragma once

#include <SDL.h>
#include "parameters.h"
#include "mylinal.h"
#include "polygon.h"
#include "rigidbody.h"
#include "lightsource.h"

float zBuff[HEIGHT][WIDTH] = { numeric_limits<float>::max() };
Vec3 directionBuff[HEIGHT][WIDTH];
Vec3 normalBuff[HEIGHT][WIDTH];
Uint32 preLightBuff[HEIGHT][WIDTH][3] = { 0 };
Uint32 postLightBuff[HEIGHT * WIDTH] = { 0 };

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

//Input variables
bool wKey(false), aKey(false), sKey(false), dKey(false), altKey(false), spaceKey(false), qKey(false), eKey(false), mouseButton(false), mouseMotion(false);
float mdx(0), mdy(0);

//Camera
struct Camera {
    SDL_Renderer* rend;
    Vec3 eye;
    float scale, pixelSize, width, height;
    const float planeDist = 100.f;
    Mat3x3 orientMat = IdMat;


    Camera(SDL_Renderer* rend, float x, float y, float z, float fov) : rend(rend), eye(x, y, z) {
        scale = WIDTH / (2.f * planeDist * tanf(fov * M_PI / 720.f));
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