#include "renderfunctions.h"

inline SDL_Point ConvertToScreen(const b2Vec2& worldCoords) {
    return { static_cast<int>(worldCoords.x * SCALE),
             static_cast<int>(-worldCoords.y * SCALE + 400) }; // Flip Y-axis and offset
}

// Render a b2Polygon with SDL2
void RenderPolygon(SDL_Renderer* renderer, const b2Polygon& polygon) {
    if (polygon.count < 2) return;

    SDL_Point points[b2_maxPolygonVertices + 1];
    for (int i = 0; i < polygon.count; ++i) {
        points[i] = ConvertToScreen(polygon.vertices[i]);
    }
    // Close the loop by connecting the last vertex to the first
    points[polygon.count] = points[0];

    // Draw the polygon outline
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // Red color
    SDL_RenderDrawLines(renderer, points, polygon.count + 1);
}




















int maining() {
    // Define a polygon (example)
    b2Polygon polygon;
    polygon.count = 4;
    polygon.vertices[0] = b2Vec2{-2.0f, 2.0f};
    polygon.vertices[1] = b2Vec2(2.0f, 2.0f);
    polygon.vertices[2] = b2Vec2(2.0f, -2.0f);
    polygon.vertices[3] = b2Vec2(-2.0f, -2.0f);

    // Main loop
    bool running = true;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }

        // Clear screen
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // Black background
        SDL_RenderClear(renderer);

        // Render polygon
        RenderPolygon(renderer, polygon);

        // Present the frame
        SDL_RenderPresent(renderer);

        SDL_Delay(16); // ~60 FPS
    }

    // Clean up
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}






















void info(){
    std::cout<<"size: "<<coins.size()<<'\n';
    std::cout<<"terrs x point: "<<terrainPoints.back().x<<'\n';
}

int main(int argc, char* args[]){
    if(!init())    {
        std::cout << "Failed to initialize!" << std::endl;
        return -1;
    }
    if(!loader())    {
        std::cout << "Failed to load!" << std::endl;
        return -1;
    }

    createCar();
    createJoin();
    generateTerrain();
    createTerrain(worldId);


    float timeStep = 1.0f / 60.0f;
    int subStepCount = 4,progress = 0;
    float segmentLength = TERRAIN_LENGTH / TERRAIN_SEGMENTS;
    bool running = true,started = false,left = false, right = false, motor = false,whog1,whog2,down = false,debug=false;;
    float cameraX = 0.0f;
    SDL_SetRenderDrawBlendMode(Rend, SDL_BLENDMODE_BLEND);
    while(running){
        Uint32 startTick;
        cameraX = b2Body_GetPosition(chasi).x- simWidth/16;

        startTick = SDL_GetTicks();
        SDL_Event ev;
        while(SDL_PollEvent(&ev) != 0){
            switch(ev.type){
                case SDL_QUIT: running = false;
                    break;
                case SDL_MOUSEBUTTONDOWN : 
                if(!started)
                    started = true;

                break;
                case SDL_KEYDOWN:
                    switch(ev.key.keysym.sym){
                        case SDLK_ESCAPE: running = false;
                            break;
                        case SDLK_LEFT:
                            // cameraX += segmentLength;
                            if(!left){
                                b2WheelJoint_SetMotorSpeed(wheel1, b2WheelJoint_GetMotorSpeed(wheel1) - 1.1f);
                                b2WheelJoint_SetMotorSpeed(wheel2, b2WheelJoint_GetMotorSpeed(wheel2) - 1.1f);
                                if(!motor){
                                    motor = true;
                                    b2WheelJoint_EnableMotor(wheel1, true);
                                    b2WheelJoint_EnableMotor(wheel2, true);
                                }
                                b2Body_ApplyAngularImpulse(chasi, 100.0f, true);
                                left = true;
                            }

                            break;
                        case SDLK_RIGHT:
                            // cameraX -= segmentLength;
                            if(!right){
                                b2WheelJoint_SetMotorSpeed(wheel1, b2WheelJoint_GetMotorSpeed(wheel1) + 1.1f);
                                b2WheelJoint_SetMotorSpeed(wheel2, b2WheelJoint_GetMotorSpeed(wheel2) + 1.1f);
                                if(!motor){
                                    motor = true;
                                    b2WheelJoint_EnableMotor(wheel1, true);
                                    b2WheelJoint_EnableMotor(wheel2, true);
                                }
                                b2Body_ApplyAngularImpulse(chasi, -100.0f, true);
                                right = true;
                            }
                            break;
                        case SDLK_UP:
                            b2Body_ApplyForceToCenter(chasi, (b2Vec2){0.0f, 10000.0f}, true);
                            // b2Body_ApplyForceToCenter(whl2, (b2Vec2){0.0f, -10000.0f}, true);
                            break;
                        case SDLK_DOWN:
                            // b2Body_ApplyForceToCenter(chasi, (b2Vec2){0.0f, -10000.0f}, true);
                            b2Body_ApplyForceToCenter(whl1, (b2Vec2){0.0f, -10000.0f}, true);
                            b2Body_ApplyForceToCenter(whl2, (b2Vec2){0.0f, -10000.0f}, true);
                            break;
                        case SDLK_d:
                            b2Body_ApplyForceToCenter(chasi, (b2Vec2){0.0f, 10.0f}, true);
                            b2Body_ApplyAngularImpulse(chasi, -50.0f, true);
                            break;
                        case SDLK_a:
                            b2Body_ApplyForceToCenter(chasi, (b2Vec2){0.0f, 10.0f}, true);
                            b2Body_ApplyAngularImpulse(chasi, 50.0f, true);
                            break;
                        case SDLK_SPACE:
                            if(!started)
                                started = true;
                            break;
                        case SDLK_RETURN:
                            started = !started;
                            break;
                        case SDLK_z:
                            down = !down;
                            break;
                        case SDLK_KP_0:
                            info();
                            break;
                        case SDLK_p:
                            debug = !debug;
                            break;
                        default:
                            break;

                    }
                    break;
                case SDL_KEYUP:
                    switch(ev.key.keysym.sym){
                        case SDLK_LEFT:
                            left = false;
                            break;
                        case SDLK_RIGHT:
                            right = false;
                            break;
                        default:
                            break;
                    }
                    break;
            }

        }
        
        SDL_SetRenderDrawColor(Rend, 0xFF, 0xFF, 0xFF, 0xFF);   //white
        SDL_RenderClear(Rend);

        renderBackground(cameraX);
        renderTerrain(Rend,cameraX);
        // SDL_RenderDrawPoint(Rend,10,10);
        renderCar(cameraX);
        showScore(cameraX);
        rendercoins(cameraX);
        showCoinCount();

        if(gameover){
            SDL_Rect overlay = {0, (SCREEN_HEIGHT-500)/2, SCREEN_WIDTH, 500};
            SDL_RenderCopy(Rend, gameoverlogo, NULL, &overlay);
        }
        else if(started){

            //             std::cout<<"wheel 1: "<< isWheelGrounded(worldId, whl1, terrainId)<<'\n';
            // std::cout<<"wheel 2: "<< isWheelGrounded(worldId, whl2, terrainId)<<"\n\n";
            whog1 = isWheelGrounded(worldId, whl1, terrainId);
            whog2 = isWheelGrounded(worldId, whl2, terrainId);

            updateTerrain(cameraX);
            if(terrendpnt - cameraX < simWidth/4)
                createTerrain(worldId);

            if(left){
                b2WheelJoint_SetMotorSpeed(wheel1, b2WheelJoint_GetMotorSpeed(wheel1) - 1.1f);
                b2WheelJoint_SetMotorSpeed(wheel2, b2WheelJoint_GetMotorSpeed(wheel2) - 1.1f);
                applyForwardForce(2000.0f,whog1,whog2,1); // negative
                if(!whog1 || !whog2)
                    b2Body_ApplyAngularImpulse(chasi, -10.0f, true);
            }
            else if(right){
                b2WheelJoint_SetMotorSpeed(wheel1, b2WheelJoint_GetMotorSpeed(wheel1) + 1.1f);
                b2WheelJoint_SetMotorSpeed(wheel2, b2WheelJoint_GetMotorSpeed(wheel2) + 1.1f);
                applyForwardForce(2000.0f,whog1,whog2,0);
                if(!whog2 || !whog1)
                    b2Body_ApplyAngularImpulse(chasi, 10.0f, true);
            }
            else if(motor){
                b2WheelJoint_SetMotorSpeed(wheel1, 0);
                b2WheelJoint_SetMotorSpeed(wheel2, 0);
                b2WheelJoint_EnableMotor(wheel1, false);
                b2WheelJoint_EnableMotor(wheel2, false);

                motor = false;
            }

            if(down){
                // b2Body_ApplyForceToCenter(chasi, (b2Vec2){0.0f, -10000.0f}, true);
                b2Body_ApplyForceToCenter(whl1, (b2Vec2){0.0f, -500.0f}, true);
                b2Body_ApplyForceToCenter(whl2, (b2Vec2){0.0f, -500.0f}, true);
            }
            // info();
            wheelstability();
            surfaceRetention(whog1,whog2);

            b2World_Step(worldId, timeStep, subStepCount);
            sensordetect();

            if(debug)
                started = false;
        }

        SDL_RenderPresent(Rend);

        Uint32 frameDuration = SDL_GetTicks() - startTick;
        // std::cout << "Frame Duration: " << frameDuration << "ms" << std::endl;
        if (frameDuration < 1000 / 60) {
                SDL_Delay((1000 / 60) - frameDuration);
            }
    }

}

