#ifndef functional
#include"setup.h"

void cleaCoins(){
    // for (int i = 0; i < coins.size(); i++)
    //     b2DestroyBody(coins[i]);
    int i=0;
    while(true){
        // if(coins.empty())
        //     break;
        if(i>=coins.size())
            break;
        
        if(b2Body_GetPosition(coins[i].coinId).x < terrainPoints.back().x){
            b2DestroyBody(coins[i].coinId);
            coins.erase(coins.begin()+i);
            std::cout<<"coin destroyed\n";
        }
        else
            i++;
    }    

    coins.clear();
}

void createTerrain() {
    if (terrainPoints.size() < 2) return;

    // Ensure points are in counterclockwise order
    for (size_t i = 1; i < terrainPoints.size(); ++i) {
        if (terrainPoints[i].x > terrainPoints[i - 1].x) {
            std::cout<<"the points are  "<<terrainPoints[i].x<<" , "<<terrainPoints[i-1].x<<'\n'<<"at i: "<<i<<'\n'; 
            std::cerr << "Error: Terrain points are not ordered correctly!\n";
            return;
        }
    }
    // Define the chain shape
    b2ChainDef chainDef = b2DefaultChainDef();
    chainDef.points = terrainPoints.data();
    chainDef.count = terrainPoints.size();
    chainDef.isLoop = false;
    chainDef.friction = 0.9f;

    // Create a static body for the terrain
    b2BodyDef terrainBodyDef = b2DefaultBodyDef();
    terrainBodyDef.type = b2_staticBody;
    b2BodyId terrainBody = b2CreateBody(worldId, &terrainBodyDef);

    // std::reverse(terrainPoints.begin(), terrainPoints.end());
    if(terrainId.index1)
    b2DestroyChain(terrainId);
    terrendpnt = terrainPoints.front().x;
    terrainId = b2CreateChain(terrainBody, &chainDef);
    // std::cout<<"terrain id: "<<terrainId.index1<<" , "<<terrainId.world0<<" , "<<terrainId.revision<<'\n';

}

void generateTerrain() {
    terrainPoints.clear();
    float segmentLength = TERRAIN_LENGTH / TERRAIN_SEGMENTS;
    float noiseScale = TERRAIN_WIDTH; // Adjust to control frequency
    float amplitude = TERRAIN_HEIGHT; // Maximum height variation

    for (int i = 0; i <= TERRAIN_SEGMENTS; ++i) {
        float x = i * segmentLength;
        float y = perlinNoise(x * noiseScale) * amplitude; // Apply Perlin noise
        terrainPoints.push_back(b2Vec2{x, y});
    }

    std::reverse(terrainPoints.begin(), terrainPoints.end());
}

void gencoin(float x, float y, int val){ 
    y+=2;
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_staticBody;
    bodyDef.position = (b2Vec2){ x , y };
    
    b2BodyId coinbod = b2CreateBody(worldId, &bodyDef);
    coins.push_back(coin{coinbod,val});

    b2Circle circle;
    circle.radius = 1;
    circle.center = (b2Vec2){0.0f, 0.0f};

    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    shapeDef.friction = 0.3f;
    shapeDef.restitution = 0.2f;
    shapeDef.isSensor = true;
    // std::cout<<"coin created at: "<<x<<" , "<<y<<'\n';
    b2CreateCircleShape(coinbod, &shapeDef, &circle);
}

void updateTerrain(float cameraX) {
    static int cointimer = 100;
    // int coinmaxdist =400;
    int coinmaxdist =200;
    int maxcoinval = 1000;
    
    
    
    float lastX = terrainPoints.front().x;

    float segmentLength = TERRAIN_LENGTH / TERRAIN_SEGMENTS;
    float noiseScale = TERRAIN_WIDTH; // Same scale as generation
    float amplitude = TERRAIN_HEIGHT;



    // Generate new points
    while (lastX - cameraX < simWidth-carpos) {
        lastX += segmentLength;
        float newY = perlinNoise(lastX * noiseScale) * amplitude;
        terrainPoints.insert(terrainPoints.begin(), b2Vec2{lastX, newY});
        if(!cointimer--){
            cointimer = rand()%coinmaxdist+30;
            // cointimer = 20;
            gencoin(lastX,newY,coinvals[rand()%maxcoinval]);
        }
    }
    while (!terrainPoints.empty() && terrainPoints.back().x < cameraX-carpos) 
        terrainPoints.pop_back();

}

b2Polygon makeCarPoly(){

    b2Vec2 points[5] = {
        {-4.0f, -2.0f},
        {-4.0f, 1.0f},
        {2.0f, 2.0f},
        {4.0f, 1.0f},
        {4.0f, -2.0f},
    };
    
    b2Hull hull = b2ComputeHull(points, 5);
    


    float radius = 0.0f; // No rounding
    return b2MakePolygon(&hull, radius);
}

void createCar( ){
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_dynamicBody;
    bodyDef.allowFastRotation = true;
    bodyDef.position = (b2Vec2){carpos-carWidth, spawn};//wheel 1
    whl1 = b2CreateBody(worldId, &bodyDef);

    bodyDef.position = (b2Vec2){carpos+carWidth, spawn};//wheel 2
    whl2 = b2CreateBody(worldId, &bodyDef);
    
    bodyDef.position = (b2Vec2){carpos, spawn+carHeight};//chasi
    bodyDef.angularDamping = 1.0f;
    bodyDef.linearDamping = 0.5f;
    bodyDef.allowFastRotation = false;
    chasi = b2CreateBody(worldId, &bodyDef);


    b2Circle circle;
    circle.radius = whlRad;
    circle.center = (b2Vec2){0.0f, 0.0f};

    b2Polygon dynamicBox = makeCarPoly();

    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 3.0f;
    shapeDef.friction = 0.3f;
    shapeDef.restitution = 0.2f;

    b2CreatePolygonShape(chasi, &shapeDef, &dynamicBox);
    shapeDef.density = 0.5f;
    shapeDef.friction = 10.9f;

    b2CreateCircleShape(whl1, &shapeDef, &circle);
    b2CreateCircleShape(whl2, &shapeDef, &circle);
}

void createJoin(){
    b2WheelJointDef wheelJointDef = b2DefaultWheelJointDef();

    wheelJointDef.bodyIdA = whl1;
    wheelJointDef.bodyIdB = chasi;
    wheelJointDef.localAnchorA = (b2Vec2){0.0f, 0.0f};
    wheelJointDef.localAnchorB = (b2Vec2){-carWidth+carWidth/6 , -(carHeight-carHeight/8)};
    wheelJointDef.localAxisA = (b2Vec2){0.0f, 1.0f}; // Ensure correct axis
    // wheelJointDef.enableMotor = true;
    wheelJointDef.enableSpring = true;
    wheelJointDef.maxMotorTorque = 1000.0f;
    // wheelJointDef.motorSpeed = -5.1f;
    wheelJointDef.hertz = 3.0f;
    wheelJointDef.dampingRatio = 0.6f;
    // wheelJointDef.enableLimit = true;
    // wheelJointDef.
    wheel1 = b2CreateWheelJoint(worldId, &wheelJointDef);

    wheelJointDef.bodyIdA = whl2;
    // wheelJointDef.bodyIdB = chasi;
    wheelJointDef.localAnchorB = (b2Vec2){carWidth-carWidth/6 , -(carHeight-carHeight/8)};

    wheel2 = b2CreateWheelJoint(worldId, &wheelJointDef);
}

void spawnWheel(float x, float y, float camx){
    x = x/SCALE + camx;
    y = y/SCALE - TERRAIN_HEIGHT;
    // std::cout<<"wheel spawned at: "<<x<<" , "<<y<<'\n';

    for(auto slot:terrainPoints){
        if(slot.x < x){
            if(slot.y > y)
                y = slot.y+2;
            break;
        }
    }

    b2BodyDef ballBodyDef = b2DefaultBodyDef();
    ballBodyDef.position = (b2Vec2){x, y};
    ballBodyDef.type = b2_dynamicBody;
    // ballBodyDef

    b2BodyId ballId = b2CreateBody(worldId, &ballBodyDef);

    b2Body_SetLinearVelocity(ballId, (b2Vec2){0.0f, 0.0f});

    b2Circle circle;
    circle.radius = dropwhlrad;
    circle.center = (b2Vec2){0.0f, 0.0f};

    b2ShapeDef ballShapeDef = b2DefaultShapeDef();
    ballShapeDef.density = 1.0f;
    ballShapeDef.friction = 0.3f;
    ballShapeDef.restitution = 0.9f;
    b2CreateCircleShape(ballId, &ballShapeDef, &circle);
    wheels.push_back(ballId);
}

bool isWheelGrounded(b2WorldId worldId, b2BodyId wheel, b2ChainId terrainChain) {
    // Get the position of the wheel
    b2Vec2 wheelPosition = b2Body_GetPosition(wheel);

    // Define the ray's direction (downward)
    b2Vec2 rayDirection = {0.0f, -1.0f}; // Downward
    float rayLength = 5.0f;              // Adjust length based on terrain scale

    // Set up the raycast input
    b2RayCastInput rayInput;
    rayInput.origin = wheelPosition;
    rayInput.translation.x = rayDirection.x * rayLength;
    rayInput.translation.y = rayDirection.y * rayLength;
    rayInput.maxFraction = 1.0f;

    // Allocate memory for terrain segments
    const int maxSegments = TERRAIN_SEGMENTS; // Adjust based on the expected number of chain segments
    b2ShapeId segmentArray[maxSegments];

    // Get segments from the terrain chain
    int segmentCount = b2Chain_GetSegments(terrainChain, segmentArray, maxSegments);

    if (segmentCount == 0) {
        std::cerr << "No terrain segments found.\n";
        return false;
    }

    // Iterate through chain segments
    for (int i = 0; i < segmentCount; ++i) {
        b2ShapeId segmentId = segmentArray[i];

        // Retrieve the chain segment
        b2ChainSegment chainSegment = b2Shape_GetChainSegment(segmentId);

        // Perform raycast against this segment
        b2CastOutput castOutput = b2RayCastSegment(&rayInput, &chainSegment.segment, false);

        if (castOutput.hit && castOutput.fraction <= 1.0f) {
            return true; // The wheel is grounded
        }
    }

    return false; // No grounding detected
}

void wheelstability(){
    float limit = 15;
    float vel = b2Body_GetAngularVelocity(whl1);
    if (abs(vel) > limit)
        b2Body_ApplyAngularImpulse(whl1, -vel, true);
    
    vel = b2Body_GetAngularVelocity(whl2);
    if (abs(vel) > limit)
        b2Body_ApplyAngularImpulse(whl2, -vel, true);
}

void surfaceRetention(bool wh1, bool wh2){ 
    b2Rot rota = b2Body_GetRotation(chasi);
    float radangel = b2Rot_GetAngle(rota);
    float angle = radangel * (180.0f / M_PI);
    // std::cout<<"angle: "<<angle<<'\n';
    if(angle > 40.0f && wh1 && wh2 && angle < 80.0f){


        b2Vec2 oppone = b2Body_GetWorldVector(whl1, (b2Vec2){-1.0f, 1.0f});

        oppone = -oppone*1000.0f;
        if(oppone.y > 0)
            oppone.y = 0;
        if(oppone.x < 0)
            oppone.x = 0;

        b2Body_ApplyForceToCenter(chasi, oppone, true);

        // std::cout<<"negative vertor: "<<oppone.x<<" , "<<oppone.y<<'\n';

    }

}

int bodyIndexer(std::vector<coin>& cnlst, b2BodyId body){
    for (int i = 0; i < cnlst.size(); i++)
        if(cnlst[i].coinId.index1 == body.index1)
            return i;
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
            int val = coins[index].value;
            coins.erase(coins.begin()+index);
            b2DestroyBody(coin);
            coincount+=val;
        }
    }
}

void removeWheel(){
    while(wheels.size()){
        b2DestroyBody(wheels.back());
        wheels.pop_back();
    }
}

void straighten(){
    float angel = b2Rot_GetAngle(b2Body_GetRotation(chasi));
    // std::cout<<"angle: "<<angel<<'\n';
    b2Body_SetAngularVelocity(chasi, -angel*2);
}

void attachHuman(){

    b2DistanceJointDef disjondef = b2DefaultDistanceJointDef();
    disjondef.bodyIdA = torso;
    disjondef.bodyIdB = chasi;
    disjondef.localAnchorA = (b2Vec2){0.0f, -bodyunit*2};
    disjondef.localAnchorB = (b2Vec2){-carWidth, 0};
    disjondef.length = 3.7f;
    disjondef.maxLength = 4.0f;
    disjondef.enableSpring = true;
    disjondef.hertz = 3.0f;
    disjondef.dampingRatio = 0.9f;

    bdj1 = b2CreateDistanceJoint(worldId, &disjondef);
    

    b2RevoluteJointDef revjoindef = b2DefaultRevoluteJointDef();
    revjoindef.bodyIdA = torso;
    revjoindef.bodyIdB = chasi;
    revjoindef.localAnchorA = (b2Vec2){0.0f, -bodyunit*1.3};
    revjoindef.localAnchorB = (b2Vec2){0.0f, bodyunit*0.5};
    bdj2 = b2CreateRevoluteJoint(worldId, &revjoindef);

}

void createhuman(){
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_dynamicBody;
    bodyDef.position = (b2Vec2){simWidth/2, spawn+carHeight+bodyunit*1.3};
    torso = b2CreateBody(worldId, &bodyDef);
    bodyDef.position = (b2Vec2){simWidth/2, spawn+carHeight+bodyunit*3};
    head = b2CreateBody(worldId, &bodyDef);

    b2Polygon dynamicBox1 = b2MakeBox(bodyunit, 2 * bodyunit);
    b2Polygon dynamicBox2 = b2MakeBox(bodyunit, bodyunit);

    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 0.1f;
    shapeDef.friction = 0.3f;
    shapeDef.restitution = 0.2f;
    shapeDef.enableContactEvents = true;

    b2CreatePolygonShape(torso, &shapeDef, &dynamicBox1);
    b2CreatePolygonShape(head, &shapeDef, &dynamicBox2);

    b2RevoluteJointDef revjoindef = b2DefaultRevoluteJointDef();
    revjoindef.bodyIdA = torso;
    revjoindef.bodyIdB = head;
    revjoindef.localAnchorA = (b2Vec2){0.0f, bodyunit*1.3};
    revjoindef.localAnchorB = (b2Vec2){0.0f, -bodyunit*0.5};
    b2CreateRevoluteJoint(worldId, &revjoindef);
    attachHuman();
}

void deathKill(){
    alive = false;
    std::cout<<"death\n";
    b2DestroyJoint(bdj1);
    b2DestroyJoint(bdj2);
}

bool dethdetect(b2BodyId bodyA,b2BodyId bodyB){
    if(bodyB.index1==head.index1){
        b2ShapeId seg[TERRAIN_SEGMENTS];
        b2Chain_GetSegments(terrainId, seg, TERRAIN_SEGMENTS);

        for(b2ShapeId i:seg)
            if(bodyA.index1==i.index1){
                deathKill();
                controlle = false;

                return true;
            }
    }
    return false;
}

void contactdetect(){
    b2ContactEvents contactEvents = b2World_GetContactEvents(worldId);
    for (int i = 0; i < contactEvents.beginCount; ++i)
    {
        // std::cout<<"contact detected\n";
        b2ContactBeginTouchEvent* beginEvent = contactEvents.beginEvents + i;
        b2BodyId coliderA = b2Shape_GetBody(beginEvent->shapeIdA);
        b2BodyId coliderB = b2Shape_GetBody(beginEvent->shapeIdB);
        // b2CollideChainSegmentAndPolygon
        if(dethdetect(coliderA,coliderB))
            break;
    }
}

#endif
