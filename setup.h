#ifndef SETUP
#include"declaration.h"

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
    std::string path = "../assets";
    std::string fpath = "../fonts";

    if (std::filesystem::exists(path)) {
        std::cout << "Directory exists: " << path << std::endl;
    } else {
        path = "./assets";
        fpath = "./assets";
        std::cout << "adjusting to : " << path << std::endl;
    }

    Font = TTF_OpenFont((fpath + "/sifi_font.ttf").c_str(), 28);
    if (Font == NULL) {
        std::cout << "Failed to load font! TTF_Error: " << TTF_GetError() << std::endl;
        return false;
    }

    carFrame = loadTexture(path+"/carFrame.png");
    // carFrame = loadTexture(path+"/carr.png");
    if (carFrame == NULL) {
        std::cout << "Failed to load carFrame texture!" << std::endl;
        return false;
    }

    wheel = loadTexture(path+"/wheel.png");
    if (wheel == NULL) {
        std::cout << "Failed to load wheel texture!" << std::endl;
        return false;
    }

    ground = loadTexture(path+"/ground.png");
    if (ground == NULL) {
        std::cout << "Failed to load ground texture!" << std::endl;
        return false;
    }

    background = loadTexture(path+"/background1.png");
    if (background == NULL) {
        std::cout << "Failed to load background texture!" << std::endl;
        return false;
    }

    coinimj = loadTexture(path+"/coin.png");
    if (coinimj == NULL) {
        std::cout << "Failed to load coin texture!" << std::endl;
        return false;
    }

    scoretxt = textTexture("SCORE: ",{20,60,200});
    if (scoretxt == NULL) {
        std::cout << "Failed to load score texture!" << std::endl;
        return false;
    }

    cointxt = textTexture("COINS: ",{20,60,200});
    if (cointxt == NULL) {
        std::cout << "Failed to load coin texture!" << std::endl;
        return false;
    }

    scoreval = textTexture("0",{20,60,200});
    if (scoreval == NULL) {
        std::cout << "Failed to load score value texture!" << std::endl;
        return false;
    }

    coinval = textTexture("0",{20,60,200});
    if (coinval == NULL) {
        std::cout << "Failed to load coin value texture!" << std::endl;
        return false;
    }

    gameoverlogo = loadTexture(path+"/Game_Over.png");
    if (gameoverlogo == NULL) {
        std::cout << "Failed to load game over texture!" << std::endl;
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
    return static_cast<int>(SCREEN_HEIGHT - ((y + height+TERRAIN_HEIGHT) * SCALE));
}


#endif