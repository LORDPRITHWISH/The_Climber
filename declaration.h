#ifndef DECLARE
#include<SDL2/SDL.h>
#include<SDL2/SDL_image.h>
#include<SDL2/SDL_ttf.h>
#include<box2d/box2d.h>
#include"perlin.h"
#include<vector>
#include<iostream>
#include <algorithm>
#include <cmath>
#include <filesystem>
// #include <box2d/b2_polygon_shape.h>

// Constants
const float SCALE = 25.0f; // Pixels per meter for rendering
const int SCREEN_WIDTH = 1440;
const int SCREEN_HEIGHT = 810;
const int SCREEN_FPS = 60;
// const int SCREEN_TICKS_PER_FRAME = 1000 / SCREEN_FPS;
const int TERRAIN_SEGMENTS = 2500;                          // Number of terrain segments

const float TERRAIN_HEIGHT = 8.0f;                        // Maximum height of terrain
const float TERRAIN_WIDTH = 0.02f;                       // noise of terrain

const float simWidth = (SCREEN_WIDTH / SCALE)*4;
const float simHeight = SCREEN_HEIGHT / SCALE;
const float carHeight = 2.0f;
const float carWidth = 4.0f;
const float whlRad = 2.2f;
const float carpos = simWidth/2;
const float spawn = simHeight/2;
const float bodyunit = 0.7f;
const float dropwhlrad = 1.5f;

// Globals
float terrendpnt = 0 ;
SDL_Window* gWindow = NULL;
SDL_Renderer* Rend = NULL;
TTF_Font* Font = NULL;
b2WorldId worldId;
SDL_Texture *carFrame= NULL, *wheel = NULL, *ground = NULL, *background = NULL, *coinimj = NULL,*scoretxt=NULL,*scoreval=NULL;
SDL_Texture *cointxt=NULL,*coinval=NULL,*gameoverlogo=NULL,*headimj=NULL,*torsoimj=NULL,*falwheel=NULL;
b2BodyId whl1,whl2,chasi,torso,head;

bool gameover = false;

b2Polygon bodygon;


std::vector<b2BodyId> wheels;
std::vector<b2BodyId> coins;
std::vector<b2Vec2> terrainPoints;
const float TERRAIN_LENGTH = simWidth;
b2ChainId terrainId;
b2JointId wheel1,wheel2;
int coincount = 0;

#endif