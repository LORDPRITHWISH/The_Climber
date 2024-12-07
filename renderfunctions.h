#ifndef RENDER
#include "functional.h"

void renderTerrain(SDL_Renderer* renderer, float cameraX) {
    SDL_SetRenderDrawColor(renderer, 34, 139, 34, 255); // Terrain color
    int segLen = static_cast<int>((TERRAIN_LENGTH / TERRAIN_SEGMENTS)*SCALE);
    int groundWidth, groundHeight;
    SDL_QueryTexture(ground, NULL, NULL, &groundWidth, &groundHeight);
    // int progress = static_cast<int>(groundWidth - (static_cast<int>(cameraX * segLen * SCALE * 9)) % groundWidth);
    int progress = (groundWidth - (static_cast<int>(cameraX*3*SCALE*0.85) % groundWidth));
    bool times = true;
    for (size_t i = 0; i < terrainPoints.size() - 1; ++i) {
        progress %= (groundWidth);
        int x = boxToScreenX(terrainPoints[i].x - cameraX);
        int y = boxToScreenY(terrainPoints[i].y);
        SDL_Rect dest = {x, y, segLen+1, SCREEN_HEIGHT - y};
        y-=100;
        SDL_Rect srct = {progress, y, (segLen), (SCREEN_HEIGHT - y)};
        progress+=(segLen*3);

        SDL_RenderCopyEx( Rend, ground, &srct, &dest, 0, NULL, SDL_FLIP_HORIZONTAL );

    }   
}

void renderCar(float cam){

    b2Vec2 boxPos = b2Body_GetPosition(chasi);
    SDL_Rect rect = {
        boxToScreenX(boxPos.x-cam, carWidth),
        boxToScreenY(boxPos.y, carHeight),
        static_cast<int>(carWidth * SCALE * 2 ),
        static_cast<int>(carHeight * SCALE * 2 )
    };
    
    SDL_Point center = {rect.w / 2, rect.h / 2};
    float angle = -b2Rot_GetAngle(b2Body_GetRotation(chasi))* (180.0f / M_PI);
    SDL_RenderCopyEx(Rend, carFrame, NULL, &rect, angle, &center, SDL_FLIP_NONE);

    boxPos = b2Body_GetPosition(whl1);
    rect = {
        boxToScreenX(boxPos.x-cam, whlRad),
        boxToScreenY(boxPos.y, whlRad),
        static_cast<int>(whlRad * SCALE * 2 ),
        static_cast<int>(whlRad * SCALE * 2 )
    };
    center = {rect.w / 2, rect.h / 2};
    angle = -b2Rot_GetAngle(b2Body_GetRotation(whl1))* (180.0f / M_PI);
    SDL_RenderCopyEx(Rend, wheel, NULL, &rect, angle, &center, SDL_FLIP_NONE);



    boxPos = b2Body_GetPosition(whl2);
    rect = {
        boxToScreenX(boxPos.x-cam, whlRad),
        boxToScreenY(boxPos.y, whlRad),
        static_cast<int>(whlRad * SCALE * 2 ),
        static_cast<int>(whlRad * SCALE * 2 )
    };
    center = {rect.w / 2, rect.h / 2};
    angle = -b2Rot_GetAngle(b2Body_GetRotation(whl2))* (180.0f / M_PI);
    SDL_RenderCopyEx(Rend, wheel, NULL, &rect, angle, &center, SDL_FLIP_NONE);


 

}

void rendercoins(float cam){
    for (size_t i = 0; i < coins.size(); ++i) {
        b2Vec2 boxPos = b2Body_GetPosition(coins[i]);
        SDL_Rect rect = {
            boxToScreenX(boxPos.x-cam, 1),
            boxToScreenY(boxPos.y, 1),
            static_cast<int>(SCALE * 2 ),
            static_cast<int>(SCALE * 2 )
        };
        SDL_RenderCopy(Rend, coinimj, NULL, &rect);
    }
}

void showScore(int camX){
    static int score = 0;
    if (score<camX-100){
        score = camX-100;
        scoreval = textTexture(std::to_string(score),{20,60,250});

    }
    SDL_Rect valrect = {20,20,140,50};
    SDL_RenderCopy(Rend, scoretxt, NULL, &valrect);
    valrect.x = 170;
    SDL_QueryTexture(scoreval,NULL,NULL,&valrect.w,&valrect.h);
    valrect.w = (valrect.w*50)/valrect.h;
    valrect.h = 50;
    
    SDL_RenderCopy(Rend, scoreval, NULL, &valrect);
}

void showCoinCount(){
    static int cointxtval = 0;
    if (cointxtval<coincount){
        cointxtval = coincount;
        coinval = textTexture(std::to_string(cointxtval),{20,60,250});
    }
    SDL_Rect valrect = {20,70,140,50};
    SDL_RenderCopy(Rend, cointxt, NULL, &valrect);
    valrect.x = 170;
    SDL_QueryTexture(coinval,NULL,NULL,&valrect.w,&valrect.h);
    valrect.w = (valrect.w*50)/valrect.h;
    valrect.h = 50;
    SDL_RenderCopy(Rend, coinval, NULL, &valrect);
}

void renderBackground(float cam){
    int backWidth, backHeight,width;
    SDL_QueryTexture(background, NULL, NULL, &backWidth, &backHeight);
    width = backWidth/backHeight*SCREEN_HEIGHT;
    // int progress = (backWidth - (static_cast<int>(cam*5) % backWidth));
    // int progress = static_cast<int>(cam) % backWidth;
    int progress = -(static_cast<int>(cam*5) % width);

    // std::cout<<"progress: "<<progress<<" cam: "<<cam<<'\n';

    // SDL_Rect rect = {progress-backWidth, 0, SCREEN_WIDTH, SCREEN_HEIGHT};
    SDL_Rect rect = {progress, 0, width, SCREEN_HEIGHT};
    SDL_RenderCopy(Rend, background, NULL, &rect);

    // std::cout<<"1st rect: "<<rect.x<<'\n';
    rect.x = progress+width;
    SDL_RenderCopy(Rend, background, NULL, &rect);

    // std::cout<<"2nd rect: "<<rect.x<<'\n';

    // std::cout<<"------------------------\n";
}

void rendWheel(float camx){
    for (size_t i = 0; i < wheels.size(); ++i) {
        b2Vec2 boxPos = b2Body_GetPosition(wheels[i]);
        SDL_Rect rect = {
            boxToScreenX(boxPos.x-camx, dropwhlrad),
            boxToScreenY(boxPos.y, dropwhlrad),
            static_cast<int>(dropwhlrad * SCALE * 2 ),
            static_cast<int>(dropwhlrad * SCALE * 2 )
        };
        SDL_Point center = {rect.w / 2, rect.h / 2};    
        float angle = -b2Rot_GetAngle(b2Body_GetRotation(wheels[i]))* (180.0f / M_PI);
        // std::cout<<"angle: "<<angle<<'\n';
        SDL_RenderCopyEx(Rend, falwheel, NULL, &rect, angle, &center, SDL_FLIP_NONE);
    }
}

void renderHuman(float camx){
    b2Vec2 boxPos = b2Body_GetPosition(torso);
    SDL_Rect rect = {
        boxToScreenX(boxPos.x-camx, bodyunit),
        boxToScreenY(boxPos.y, 2*bodyunit),
        static_cast<int>(bodyunit * SCALE * 2 ),
        static_cast<int>(2 * bodyunit * SCALE * 2 )
    };
    
    SDL_Point center = {rect.w / 2, rect.h / 2};
    float angle = -b2Rot_GetAngle(b2Body_GetRotation(torso))* (180.0f / M_PI);
    SDL_RenderCopyEx(Rend, torsoimj, NULL, &rect, angle, &center, SDL_FLIP_NONE);

    boxPos = b2Body_GetPosition(head);
    rect = {
        boxToScreenX(boxPos.x-camx, bodyunit),
        boxToScreenY(boxPos.y, bodyunit),
        static_cast<int>(bodyunit * SCALE * 2 ),
        static_cast<int>(bodyunit * SCALE * 2 )
    };
    center = {rect.w / 2, rect.h / 2};
    angle = -b2Rot_GetAngle(b2Body_GetRotation(head))* (180.0f / M_PI);
    SDL_RenderCopyEx(Rend, headimj, NULL, &rect, angle, &center, SDL_FLIP_NONE);

}


#endif