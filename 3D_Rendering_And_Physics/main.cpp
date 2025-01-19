#include <SDL.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <chrono>
#include <vector>
#include <limits>
#include <algorithm>
#include "parameters.h"
#include "mylinal.h"
#include "polygon.h"
#include "rigidbody.h"
#include "camera.h"

using namespace std;


//Main loop variables
const long long tickCap = int(1000 / FPS);
long long start, tickTime;
SDL_Event event;
int tickCnt = 0;
bool quit = false;
auto t1 = chrono::system_clock::now().time_since_epoch();
auto t2 = chrono::system_clock::now().time_since_epoch();


//Main
int main(int argc, char* args[]) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        return -1;
    }
    SDL_Window* window = SDL_CreateWindow("SDL Tutorial", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);
    SDL_Renderer* rend = SDL_CreateRenderer(window, 0, SDL_RENDERER_ACCELERATED);
    SDL_Texture* texture = SDL_CreateTexture(rend, SDL_PIXELFORMAT_RGB888, SDL_TEXTUREACCESS_STREAMING, WINDOW_WIDTH, WINDOW_HEIGHT);


    //Creating camera
    Camera cam(rend, CAM_INIT_X, CAM_INIT_Y, CAM_INIT_Z, FOV);
    cam.rotSelfOX(-M_PI / 2);


    //Creating objects
    RigidBody icosahedron = createIcosahedron(1e-4, 20.f);
    icosahedron.bodyMove(Vec3(125, 0, 0));

    RigidBody hammerHead = createCuboid(1e-4, 50, 100, 50);
    hammerHead.colorPoly(4, 255, 0, 0); hammerHead.colorPoly(5, 255, 0, 0);
    hammerHead.colorPoly(6, 0, 0, 255); hammerHead.colorPoly(7, 0, 0, 255);
    RigidBody hammerHandle = createCuboid(1e-4, 100, 10, 10);
    hammerHandle.bodyMove(Vec3(75, 0, 0));
    RigidBody hammer = glueTogether(hammerHead, hammerHandle);
    hammer = glueTogether(hammer, icosahedron);
    hammer.bodyMove(-hammer.cmPos);

    hammer.angMom = Vec3(0, 15000, 0.01);

    vector<LightSource> lights;
    LightSource light1(0, 0, 300, 40000);
    lights.push_back(light1);


    //Main loop
    while (not quit) {
        start = chrono::system_clock::now().time_since_epoch() / chrono::milliseconds(1);
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
        cam.applyLight(lights, texture);
        t2 = chrono::system_clock::now().time_since_epoch();
        cout << (t2 - t1) / chrono::milliseconds(1) << "    ";


        t1 = chrono::system_clock::now().time_since_epoch();
        cam.draw(texture);
        SDL_RenderPresent(rend);
        t2 = chrono::system_clock::now().time_since_epoch();
        cout << (t2 - t1) / chrono::milliseconds(1) << "\n";

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


    SDL_DestroyWindow(window);
    SDL_DestroyRenderer(rend);
    SDL_Quit();

    return 0;
}