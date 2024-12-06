#include "renderfunctions.h"

int bodyIndexer(std::vector<b2BodyId>& cnlst, b2BodyId body){
        // b2Vec2 speed = b2Body_GetLinearVelocity(body);
        // std::cout<<speed.x<<" , "<<speed.y <<'\n';
    for (int i = 0; i < cnlst.size(); i++){
        // if(cnlst[i].index1 == body.index1 && cnlst[i].world0 == body.world0 && cnlst[i].revision == body.revision)

        // std::cout<<"coin: "<<cnlst[i].index1<<" vs "<<body.index1<<'\n';

        if(cnlst[i].index1 == body.index1)
            return i;
    }
    return -1;
}

void sensordetect(){
    b2SensorEvents sensorEvents = b2World_GetSensorEvents(worldId);
    for (int i = 0; i < sensorEvents.beginCount; ++i)
    {
        b2SensorBeginTouchEvent* beginTouch = sensorEvents.beginEvents + i;
        b2BodyId coin = b2Shape_GetBody(beginTouch->sensorShapeId);
        int index = bodyIndexer(coins, coin);
        if(index != -1){
            coins.erase(coins.begin()+index);
            // b2DestroyBody(coin);
        }

        // void* myUserData = b2Shape_GetUserData(beginTouch->visitorShapeId);
        // std::cout << "Sensor Begin Event: " << myUserData << std::endl;
        // process begin event
    }
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
    int subStepCount = 4;
    float segmentLength = TERRAIN_LENGTH / TERRAIN_SEGMENTS;
    int progress = 0;
    bool running = true,started = false;
    float cameraX = 0.0f;
    bool left = false, right = false, motor = false,whog1,whog2,down = false,debug=false;
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
        
        // SDL_SetRenderDrawColor(Rend, 0, 0, 0, 0xFF);     //black
        SDL_SetRenderDrawColor(Rend, 0xFF, 0xFF, 0xFF, 0xFF);   //white
        SDL_RenderClear(Rend);
        SDL_RenderCopy(Rend, background, NULL, NULL);
        renderTerrain(Rend,cameraX);
        // SDL_RenderDrawPoint(Rend,10,10);
        renderCar(cameraX);
        if(started){

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
        showScore(cameraX);
        rendercoins(cameraX);
        SDL_RenderPresent(Rend);

        Uint32 frameDuration = SDL_GetTicks() - startTick;
        // std::cout << "Frame Duration: " << frameDuration << "ms" << std::endl;
        if (frameDuration < 1000 / 60) {
                SDL_Delay((1000 / 60) - frameDuration);
            }
    }

}

