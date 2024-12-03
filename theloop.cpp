theloop

#include <box2d/box2d.h>
#include <SDL2/SDL.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <SDL2/SDL_image.h>
#include <cmath>
#include <algorithm>


const float SCALE = 20.0f; // Pixels per meter for rendering
const int SCREEN_WIDTH = 1440;
const int SCREEN_HEIGHT = 810;

const float simWidth = SCREEN_WIDTH/SCALE;
const float simHeight = SCREEN_HEIGHT/SCALE;

const float grdWidth = simWidth;
const float grdHeight = simHeight*0.01;

const float whlRad = 1.0f;

const float chaHeight = 2.0f;
const float chaWidth = 4.0f;

float carpos = simHeight/4;


const int TERRAIN_SEGMENTS = 100; // Number of segments visible on screen
const float TERRAIN_LENGTH = simWidth; // Length of visible terrain in simulation units
const float TERRAIN_HEIGHT = 5.0f; // Maximum height variation


void createTerrain(b2WorldId worldId) {
    if (terrainPoints.size() < 2) return;

    // Ensure points are in counterclockwise order
    for (size_t i = 1; i < terrainPoints.size(); ++i) {
        if (terrainPoints[i].x < terrainPoints[i - 1].x) {
            std::cerr << "Error: Terrain points are not ordered correctly!\n";
            return;
        }
    }

    // Define the chain shape
    b2ChainDef chainDef = b2DefaultChainDef();
    chainDef.points = terrainPoints.data();
    chainDef.count = terrainPoints.size();
    chainDef.isLoop = false;

    // Create a static body for the terrain
    b2BodyDef terrainBodyDef = b2DefaultBodyDef();
    terrainBodyDef.type = b2_staticBody;
    b2BodyId terrainBody = b2CreateBody(worldId, &terrainBodyDef);

    std::reverse(terrainPoints.begin(), terrainPoints.end());


    // Attach the chain shape to the body
    b2CreateChain(terrainBody, &chainDef);

    std::cout << "Created terrain with " << terrainPoints.size() << " points.\n";
}


void updateTerrain(float cameraX, b2WorldId worldId) {
    // Ensure terrainPoints isn't empty
    if (terrainPoints.empty()) return;

    // Generate new terrain segments if needed
    float lastX = terrainPoints.back().x;
    while (lastX - cameraX < simWidth) {
        lastX += TERRAIN_LENGTH / TERRAIN_SEGMENTS;
        float newY = sinf(lastX * 0.5f) * TERRAIN_HEIGHT;
        terrainPoints.push_back(b2Vec2{lastX, newY});
    }

    // Remove old segments
    while (terrainPoints.size() > TERRAIN_SEGMENTS + 1) {
        terrainPoints.erase(terrainPoints.begin());
    }

    // Rebuild the chain shape only if necessary
    createTerrain(worldId);
}

void SDL_RenderDrawCircle(SDL_Renderer* renderer, int x, int y, int radius) {
    int offsetX, offsetY, d;
    int status;

    offsetX = 0;
    offsetY = radius;
    d = radius - 1;
    status = 0;

    while (offsetY >= offsetX) {
        if (x + offsetX >= 0 && x + offsetX < SCREEN_WIDTH && y + offsetY >= 0 && y + offsetY < SCREEN_HEIGHT) {
            SDL_RenderDrawLine(renderer, x - offsetY, y + offsetX, x + offsetY, y + offsetX);
            SDL_RenderDrawLine(renderer, x - offsetX, y + offsetY, x + offsetX, y + offsetY);
            SDL_RenderDrawLine(renderer, x - offsetX, y - offsetY, x + offsetX, y - offsetY);
            SDL_RenderDrawLine(renderer, x - offsetY, y - offsetX, x + offsetY, y - offsetX);
        }


        if (d >= 2 * offsetX) {
            d -= 2 * offsetX + 1;
            offsetX += 1;
        } else if (d < 2 * (radius - offsetY)) {
            d += 2 * offsetY - 1;
            offsetY -= 1;
        } else {
            d += 2 * (offsetY - offsetX - 1);
            offsetY -= 1;
            offsetX += 1;
        }
    }
}


int main() {


    // Ground body
    b2BodyDef groundBodyDef = b2DefaultBodyDef();
    // groundBodyDef.position = (b2Vec2){0.0f+grdWidth, 0.0f+grdHeight};
    groundBodyDef.position = (b2Vec2){0.0f+grdWidth, 0.0f+grdHeight};

    b2BodyId groundId = b2CreateBody(worldId, &groundBodyDef);
    b2Polygon groundBox = b2MakeBox(grdWidth, grdHeight);

    b2ShapeDef groundShapeDef = b2DefaultShapeDef();
    groundShapeDef.density = 1.0f;
    groundShapeDef.friction = 0.3f;
    groundShapeDef.restitution = 0.2f;
    b2CreatePolygonShape(groundId, &groundShapeDef, &groundBox);

    // Simulation loop
    while (running) {
        // Event handling
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
            float mult = 100;
            if (event.type == SDL_KEYDOWN) {
                b2Vec2 velocity = b2Body_GetLinearVelocity(whl1);
                int moux, mouy;
                switch (event.key.keysym.sym) {
                    case SDLK_LEFT:
                        b2Body_ApplyLinearImpulseToCenter(whl1, (b2Vec2){-mult, 0.0f}, true);
                        break;
                    case SDLK_RIGHT:
                        b2Body_ApplyLinearImpulseToCenter(whl2, (b2Vec2){mult, 0.0f}, true);
                        break;
                    case SDLK_UP:
                    if (velocity.y <= 5.0f){
                        b2Body_ApplyLinearImpulseToCenter(whl1, (b2Vec2){0.0f, mult}, true);
                        b2Body_ApplyLinearImpulseToCenter(whl2, (b2Vec2){0.0f, mult}, true);
                    }
                        break;
                    case SDLK_DOWN:
                        b2Body_ApplyLinearImpulseToCenter(whl1, (b2Vec2){0.0f, -mult}, true);
                        b2Body_ApplyLinearImpulseToCenter(whl2, (b2Vec2){0.0f, -mult}, true);
                        break;
                    case SDLK_SPACE:
                        if (abs(velocity.y) <= 15.0f) { // Ensure the box is stationary vertically
                            b2Vec2 jumpImpulse = {0.0f, mult*10}; // Impulse vector (x, y)
                            b2Body_ApplyLinearImpulseToCenter(chassi, jumpImpulse, true);
                        }
                        break;
                    default:
                        break;
                }

            }

        }
        // Step the physics world
        b2World_Step(worldId, timeStep, subStepCount);
        

        // Get positions
        b2Vec2 groundPos = b2Body_GetPosition(groundId);
        b2Vec2 dynamicPos1 = b2Body_GetPosition(whl1);
        b2Vec2 dynamicPos2 = b2Body_GetPosition(whl2);
        b2Vec2 dynamicPosch = b2Body_GetPosition(chassi);

        // Clear the screen
        // SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        // SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 135, 206, 235, 255); // Sky blue
        SDL_RenderClear(renderer);

        // Draw the ground
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_Rect groundRect = {
            worldToScreenX(groundPos.x, grdWidth),
            worldToScreenY(groundPos.y, grdHeight),
            static_cast<int>(grdWidth * SCALE * 2),
            static_cast<int>(grdHeight * SCALE * 2)
        };
        SDL_RenderFillRect(renderer, &groundRect);

        // Draw the dynamic box
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        int circleX = worldToScreenX(dynamicPos1.x);
        int circleY = worldToScreenY(dynamicPos1.y);
        int circleRadius = static_cast<int>(whlRad * SCALE);

        SDL_RenderDrawCircle(renderer, circleX, circleY, circleRadius);


        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        circleX = worldToScreenX(dynamicPos2.x, 0);
        circleY = worldToScreenY(dynamicPos2.y, 0);
        circleRadius = static_cast<int>(whlRad * SCALE);

        SDL_RenderDrawCircle(renderer, circleX, circleY, circleRadius);

        b2Vec2 carPos = b2Body_GetPosition(chassi);

        float cameraX = carPos.x - simWidth / 4.0f;
        updateTerrain(cameraX, worldId);
        renderTerrain(renderer, cameraX);




        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        // circleX = worldToScreenX(dynamicPos2.x, 0);
        // circleY = worldToScreenY(dynamicPos2.y, 0);
        // circleRadius = static_cast<int>(dynWidth * SCALE);
        SDL_Rect rect = {
            worldToScreenX(dynamicPosch.x, chaWidth),
            worldToScreenY(dynamicPosch.y, chaHeight),
            static_cast<int>(chaWidth * SCALE * 2),
            static_cast<int>(chaHeight * SCALE * 2)
        };
        b2Rot rotation = b2Body_GetRotation(chassi);
        // b2Vec2 xax = b2Rot_GetXAxis(rotation);
        // b2Vec2 yax = b2Rot_GetYAxis(rotation);
        // float xx = xax.x;
        // float xy = xax.y;
        // float yx = yax.x;
        // float yy = yax.y;

        SDL_Point center = {rect.w / 2, rect.h / 2};

        float angel = b2Rot_GetAngle(rotation);
        float angleInDegrees = -angel * (180.0f / M_PI);
        // std::cout << angel << std::endl;
        
        SDL_RenderCopyEx(renderer, chasetx, NULL, &rect, angleInDegrees, &center, SDL_FLIP_NONE);

        // printf("angle: %f\n", angel);


        SDL_RenderDrawRect(renderer, &rect);





        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);

        // Present the updated screen
        SDL_RenderPresent(renderer);

        SDL_Delay(16); // ~60 FPS
    }

    // Clean up
    b2DestroyWorld(worldId);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_DestroyTexture(chasetx);
    SDL_Quit();

    return 0;
}
