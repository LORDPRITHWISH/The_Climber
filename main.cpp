#include<SDL2/SDL.h>
#include<SDL2/SDL_image.h>
#include<SDL2/SDL_ttf.h>
#include<box2d/box2d.h>
#include"perlin.h"
#include<vector>
#include<iostream>
#include <algorithm>
#include <cmath>

// Constants
const float SCALE = 20.0f; // Pixels per meter for rendering
const int SCREEN_WIDTH = 1440;
const int SCREEN_HEIGHT = 810;
const int SCREEN_FPS = 60;
// const int SCREEN_TICKS_PER_FRAME = 1000 / SCREEN_FPS;
const int TERRAIN_SEGMENTS = 1000;                       // Number of terrain segments

const float TERRAIN_HEIGHT = 4.0f;                      // Maximum height of terrain
const float TERRAIN_WIDTH = 0.1f;                       // noise of terrain

const float simWidth = SCREEN_WIDTH / SCALE;
const float simHeight = SCREEN_HEIGHT / SCALE;
const float carHeight = 2.0f;
const float carWidth = 4.0f;
const float whlRad = 2.0f;
const float carpos = simWidth/4;
const float spawn = simHeight/2;

// Globals
SDL_Window* gWindow = NULL;
SDL_Renderer* Rend = NULL;
TTF_Font* Font = NULL;
b2WorldId worldId;
SDL_Texture* carFrame= NULL, *wheel = NULL, *ground = NULL,*background = NULL;
b2BodyId whl1,whl2,chasi,anchor,axil;

std::vector<b2Vec2> terrainPoints;
const float TERRAIN_LENGTH = simWidth;


SDL_Texture* loadTexture(const std::string& path) {
    SDL_Texture* newTexture = IMG_LoadTexture(Rend, path.c_str());
    if (newTexture == NULL) {
        std::cout << "Failed to load texture! IMG_Error: " << IMG_GetError() << std::endl;
    }
    return newTexture;
}
SDL_Texture* textTexture(const std::string& text,SDL_Color color = {0,0,0}) {
    SDL_Surface* textsurf = TTF_RenderText_Solid( Font, text.c_str(), color );
    if ( !textsurf ) {
        std::cout << "Failed to render text: " << TTF_GetError() << std::endl;
    }

    SDL_Texture* texTure = SDL_CreateTextureFromSurface( Rend, textsurf );

    return texTure;
}

// Function prototypes
bool init(){
    if(SDL_Init(SDL_INIT_EVERYTHING) < 0)    {
        std::cout << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
        return false;
    }

    if(SDL_CreateWindowAndRenderer(SCREEN_WIDTH,SCREEN_HEIGHT,0,&gWindow,&Rend) < 0)    {
        std::cout << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        return false;
    }

    if (!(IMG_Init(IMG_INIT_PNG) & IMG_INIT_PNG)) {
        std::cout << "Failed to initialize SDL_image! IMG_Error: " << IMG_GetError() << std::endl;
        return false;
    }

    if ( TTF_Init() < 0 ) {
	    std::cout << "Error initializing SDL_ttf: " << TTF_GetError() << std::endl;
    }

    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity = (b2Vec2){0.0f, -10.0f};
    worldId = b2CreateWorld(&worldDef);

    return true;


}
bool loader(){
    Font = TTF_OpenFont("../fonts/sifi_font.ttf", 28);
    if (Font == NULL) {
        std::cout << "Failed to load font! TTF_Error: " << TTF_GetError() << std::endl;
        return false;
    }

    carFrame = loadTexture("../assets/carFrame.png");
    if (carFrame == NULL) {
        std::cout << "Failed to load carFrame texture!" << std::endl;
        return false;
    }

    wheel = loadTexture("../assets/wheel.png");
    if (wheel == NULL) {
        std::cout << "Failed to load wheel texture!" << std::endl;
        return false;
    }

    ground = loadTexture("../assets/ground.jpg");
    if (ground == NULL) {
        std::cout << "Failed to load ground texture!" << std::endl;
        return false;
    }

    background = loadTexture("../assets/background1.png");
    if (background == NULL) {
        std::cout << "Failed to load background texture!" << std::endl;
        return false;
    }

    return true;

}
void closer(){
    SDL_DestroyTexture(carFrame);
    SDL_DestroyTexture(wheel);
    SDL_DestroyTexture(ground);
    SDL_DestroyTexture(background);
    SDL_DestroyRenderer(Rend);
    SDL_DestroyWindow(gWindow);
    TTF_CloseFont(Font);
    TTF_Quit();
    IMG_Quit();
    SDL_Quit();
}

int boxToScreenX(float x, float width = 0.0f) {
    return static_cast<int>((x - width) * SCALE );
}
int boxToScreenY(float y, float height = 0.0f) {
    return static_cast<int>(SCREEN_HEIGHT - ((y + height) * SCALE));
}

void createTerrain(b2WorldId worldId) {
    if (terrainPoints.size() < 2) return;

    // Ensure points are in counterclockwise order
    for (size_t i = 1; i < terrainPoints.size(); ++i) {
        if (terrainPoints[i].x < terrainPoints[i - 1].x-0.1) {
            std::cout<<"the points are  "<<terrainPoints[i].x<<" , "<<terrainPoints[i-1].x<<'\n'<<"at i: "<<i<<'\n'; 
            std::cerr << "Error: Terrain points are not ordered correctly!\n";
            // printf("i: %d\n",i);
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

    // std::cout << "Created terrain with " << terrainPoints.size() << " points.\n";
}


void generateTerrain() {
    // printf("Generating terrain...\n");
    terrainPoints.clear();
    float segmentLength = TERRAIN_LENGTH / TERRAIN_SEGMENTS;
    float noiseScale = TERRAIN_WIDTH; // Adjust to control frequency
    float amplitude = TERRAIN_HEIGHT; // Maximum height variation

    for (int i = 0; i <= TERRAIN_SEGMENTS; ++i) {
        float x = i * segmentLength;
        float y = perlinNoise(x * noiseScale) * amplitude; // Apply Perlin noise
        terrainPoints.push_back(b2Vec2{x, y});
    }
    std::cout<<"terrainPoints: "<<terrainPoints.size()<<'\n';
    std::cout<<"begin: "<<terrainPoints.front().x<<'\n';
    std::cout<<"end: "<<terrainPoints.back().x<<'\n';
    // std::reverse(terrainPoints.begin(), terrainPoints.end());
}
void updateTerrain(float cameraX) {
    float lastX = terrainPoints.back().x;
    float segmentLength = TERRAIN_LENGTH / TERRAIN_SEGMENTS;
    float noiseScale = TERRAIN_WIDTH; // Same scale as generation
    float amplitude = TERRAIN_HEIGHT;

    int ran=0;

    // Generate new points
    while (lastX - cameraX < simWidth) {
        lastX += segmentLength;
        float newY = perlinNoise(lastX * noiseScale) * amplitude;
        terrainPoints.push_back(b2Vec2{lastX, newY});
        ran++;
        // printf("xxxxxxxxxxxxlastX: %f\n", lastX);
        // terrainPoints.insert(terrainPoints.begin(), b2Vec2{lastX, newY});
    }
    if(ran)
    std::cout<<"ran: "<<ran<<'\n';

    // std::cout<<"lastX: "<<lastX<<'\n';
    // std::cout<<"cameraX: "<<cameraX<<'\n';
    // std::cout<<"simWidth: "<<simWidth<<'\n';
    // std::cout<<"terrainPoints: "<<terrainPoints.size()<<'\n';
    // std::cout<<"diff: "<<lastX-cameraX<<'\n';
    // std::cout<<"TERRAIN_LENGTH: "<<TERRAIN_LENGTH<<'\n';
    // std::cout<<"lasrY: "<<terrainPoints.back().y<<'\n'<<"firstY: "<<terrainPoints.front().y<<'\n';
    // std::cout<<"segmentLength: "<<segmentLength<<"\n\n\n";
    // std::cout<<"-----------------------------------\n\n\n";
    // // Remove old points
    // std::cout<<"front: "<<terrainPoints.front().x<<'\n';
    // std::cout<<"back: "<<terrainPoints.back().x<<'\n';
    // std::cout<<"cameraX: "<<cameraX<<'\n';
    // std::cout<<"simWidth: "<<simWidth<<'\n';
    // std::cout<<"diff: "<<cameraX - simWidth<<'\n';


    while (!terrainPoints.empty() && terrainPoints.back().x < cameraX - simWidth) {
        terrainPoints.erase(terrainPoints.begin());
        std::cout<<"erased\n";
        // terrainPoints.pop_back();
    }
}

int renderTerrain(SDL_Renderer* renderer, float cameraX, int progress = 0) {
    SDL_SetRenderDrawColor(renderer, 34, 139, 34, 255); // Terrain color
    int segLen = static_cast<int>((TERRAIN_LENGTH / TERRAIN_SEGMENTS)*SCALE);
    // int unit = segLen
    
    // int progress = cameraX * segLen;
    // int progress = (terrainPoints[i].x - cameraX) * SCALE;
    int groundWidth, groundHeight;
    SDL_QueryTexture(ground, NULL, NULL, &groundWidth, &groundHeight);
    for (size_t i = 0; i < terrainPoints.size() - 1; ++i) {
        int x = boxToScreenX(terrainPoints[i].x - cameraX);
        int y = boxToScreenY(terrainPoints[i].y);
        // int x2 = boxToScreenX(terrainPoints[i + 1].x - cameraX);
        // int y2 = boxToScreenY(terrainPoints[i + 1].y);

        SDL_Rect dest = {x, y, segLen+1, SCREEN_HEIGHT - y};

        SDL_Rect srct = {progress, y/4, (segLen*4), (SCREEN_HEIGHT - y)*4};
        progress+=(segLen*4);
        // progress++;
        // if(progress >= groundWidth)
        // progress = 0;
        progress %= (groundWidth/segLen);
        // std::cout<<"x: "<<x<<'\n';
        // std::cout<<"y: "<<y<<'\n';
        // std::cout << "Progress: " << progress << std::endl;
        // std::cout << "Ground Width: " << groundWidth << std::endl;
        // std::cout << "i: " << i << std::endl;
        // // SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
        SDL_RenderCopy(Rend,ground,&srct,&dest);
    }
        // std::cout << "Terrain Points: " << terrainPoints.size() << std::endl;
        // std::cout << "segLen: " << segLen << std::endl;
        return progress;
}

void createCar( ){
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_dynamicBody;
    bodyDef.position = (b2Vec2){carpos-carWidth, spawn};//wheel 1
    whl1 = b2CreateBody(worldId, &bodyDef);

    bodyDef.position = (b2Vec2){carpos+carWidth, spawn};//wheel 2
    whl2 = b2CreateBody(worldId, &bodyDef);
    
    bodyDef.position = (b2Vec2){carpos, spawn+carHeight};//chasi
    chasi = b2CreateBody(worldId, &bodyDef);


    b2Circle circle;
    circle.radius = whlRad;
    circle.center = (b2Vec2){0.0f, 0.0f};
    b2Polygon dynamicBox = b2MakeBox(carWidth, carHeight);

    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 3.0f;
    shapeDef.friction = 0.3f;
    shapeDef.restitution = 0.3f;
    b2CreatePolygonShape(chasi, &shapeDef, &dynamicBox);
    shapeDef.density = 1.0f;

    b2CreateCircleShape(whl1, &shapeDef, &circle);
    b2CreateCircleShape(whl2, &shapeDef, &circle);


    b2BodyDef anchorDef = b2DefaultBodyDef();
    anchorDef.type = b2_staticBody;
    anchorDef.position = b2Body_GetPosition(chasi); // Anchor at car's current position
    anchor = b2CreateBody(worldId, &anchorDef);


    b2BodyDef axilDef = b2DefaultBodyDef();
    axilDef.type = b2_dynamicBody; // Correctly define as dynamic
    axilDef.position = b2Body_GetPosition(chasi); // Place axil at chassis' position
    axil = b2CreateBody(worldId, &axilDef);

    std::cout<<"the type is : "<<anchorDef.type<<'\n';

    b2ShapeDef axilShapeDef = b2DefaultShapeDef();
    axilShapeDef.density = 1.0f; // Ensure non-zero density
    axilShapeDef.friction = 0.3f;
    axilShapeDef.restitution = 0.1f;
    b2Polygon axilBox = b2MakeBox(0.1f, 0.1f); // Small box for axil
    b2CreatePolygonShape(axil, &axilShapeDef, &axilBox);
}

void createJoin(){
    b2WheelJointDef wheelJointDef = b2DefaultWheelJointDef();

    wheelJointDef.bodyIdA = whl1;
    wheelJointDef.bodyIdB = chasi;
    wheelJointDef.localAnchorA = (b2Vec2){0.0f, 0.0f};
    wheelJointDef.localAnchorB = (b2Vec2){-carWidth+carWidth/6 , -(carHeight-carHeight/8)};
    wheelJointDef.localAxisA = (b2Vec2){0.0f, 1.0f}; // Ensure correct axis
    wheelJointDef.enableMotor = true;
    wheelJointDef.enableSpring = true;
    wheelJointDef.maxMotorTorque = 10000.0f;
    wheelJointDef.motorSpeed = 4.0f;
    wheelJointDef.hertz = 3.0f;
    wheelJointDef.dampingRatio = 0.8f;

    b2CreateWheelJoint(worldId, &wheelJointDef);

    wheelJointDef.bodyIdA = whl2;
    // wheelJointDef.bodyIdB = chasi;
    wheelJointDef.localAnchorB = (b2Vec2){carWidth-carWidth/6 , -(carHeight-carHeight/8)};

    b2CreateWheelJoint(worldId, &wheelJointDef);

    b2PrismaticJointDef prismaticDef = b2DefaultPrismaticJointDef();
    prismaticDef.bodyIdA = anchor;
    prismaticDef.bodyIdB = axil;
    prismaticDef.localAnchorA = (b2Vec2){0.0f, 0.0f};
    prismaticDef.localAnchorB = (b2Vec2){0.0f, 0.0f};
    prismaticDef.localAxisA = (b2Vec2){0.0f, 1.0f}; // Restrict to vertical movement
    // prismaticDef.enableLimit = true;
    prismaticDef.lowerTranslation = -simHeight; // Allow vertical movement within bounds
    prismaticDef.upperTranslation = simHeight;
    b2CreatePrismaticJoint(worldId, &prismaticDef);

    b2RevoluteJointDef revolveDef = b2DefaultRevoluteJointDef();
    revolveDef.bodyIdA = axil;
    revolveDef.bodyIdB = chasi;
    revolveDef.localAnchorA = (b2Vec2){0.0f, 0.0f};
    revolveDef.localAnchorB = (b2Vec2){0.0f, 0.0f};
    b2CreateRevoluteJoint(worldId, &revolveDef);

    

}

void renderCar(){

    b2Vec2 boxPos = b2Body_GetPosition(chasi);
    SDL_Rect rect = {
        boxToScreenX(boxPos.x, carWidth),
        boxToScreenY(boxPos.y, carHeight),
        static_cast<int>(carWidth * SCALE * 2 ),
        static_cast<int>(carHeight * SCALE * 2 )
    };
    SDL_Point center = {rect.w / 2, rect.h / 2};
    float angle = -b2Rot_GetAngle(b2Body_GetRotation(chasi))* (180.0f / M_PI);
    SDL_RenderCopyEx(Rend, carFrame, NULL, &rect, angle, &center, SDL_FLIP_NONE);

    boxPos = b2Body_GetPosition(whl1);
    rect = {
        boxToScreenX(boxPos.x, whlRad),
        boxToScreenY(boxPos.y, whlRad),
        static_cast<int>(whlRad * SCALE * 2 ),
        static_cast<int>(whlRad * SCALE * 2 )
    };
    center = {rect.w / 2, rect.h / 2};
    angle = -b2Rot_GetAngle(b2Body_GetRotation(whl1))* (180.0f / M_PI);
    SDL_RenderCopyEx(Rend, wheel, NULL, &rect, angle, &center, SDL_FLIP_NONE);


    boxPos = b2Body_GetPosition(whl2);
    rect = {
        boxToScreenX(boxPos.x, whlRad),
        boxToScreenY(boxPos.y, whlRad),
        static_cast<int>(whlRad * SCALE * 2 ),
        static_cast<int>(whlRad * SCALE * 2 )
    };
    center = {rect.w / 2, rect.h / 2};
    angle = -b2Rot_GetAngle(b2Body_GetRotation(whl2))* (180.0f / M_PI);
    SDL_RenderCopyEx(Rend, wheel, NULL, &rect, angle, &center, SDL_FLIP_NONE);



 

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



    float timeStep = 1.0f / 60.0f;
    int subStepCount = 4;
    createJoin();
    generateTerrain();
    createTerrain(worldId);
    float segmentLength = TERRAIN_LENGTH / TERRAIN_SEGMENTS;

    Uint32 startTick;
    int progress = 0;
    bool running = true,started = false;
    float cameraX = 0.0f;
    while(running){
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
                            cameraX += segmentLength;
                            // b2Body_ApplyForceToCenter(whl1, (b2Vec2){-1000.0f, 0.0f}, true);
                            // b2Body_ApplyForceToCenter(whl2, (b2Vec2){-1000.0f, 0.0f}, true);
                            break;
                        // case SDLK_RIGHT:
                        //     b2Body_ApplyForceToCenter(whl1, (b2Vec2){1000.0f, 0.0f}, true);
                        //     b2Body_ApplyForceToCenter(whl2, (b2Vec2){1000.0f, 0.0f}, true);
                        //     break;
                        // case SDLK_UP:
                        //     b2Body_ApplyForceToCenter(whl1, (b2Vec2){0.0f, -1000.0f}, true);
                        //     b2Body_ApplyForceToCenter(whl2, (b2Vec2){0.0f, -1000.0f}, true);
                        //     break;
                        // case SDLK_DOWN:
                        //     b2Body_ApplyForceToCenter(whl1, (b2Vec2){0.0f, 1000.0f}, true);
                        //     b2Body_ApplyForceToCenter(whl2, (b2Vec2){0.0f, 1000.0f}, true);
                        //     break;
                        case SDLK_SPACE:
                            if(!started)
                                started = true;
                            break;
                        case SDLK_RETURN:
                            started = !started;
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
        progress = renderTerrain(Rend, 0.0f,progress);
        // SDL_RenderDrawPoint(Rend,10,10);
        renderCar();
        if(started){
            updateTerrain(cameraX);
            createTerrain(worldId);
            // printf("CameraX: %f\n", cameraX);
            b2World_Step(worldId, timeStep, subStepCount);
        }
            SDL_RenderPresent(Rend);

            Uint32 frameDuration = SDL_GetTicks() - startTick;
            // std::cout << "Frame Duration: " << frameDuration << "ms" << std::endl;
            if (frameDuration < 1000 / 60) {
                SDL_Delay((1000 / 60) - frameDuration);
            }
    }

}

