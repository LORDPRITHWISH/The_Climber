#ifndef functional
#include"setup.h"


void createTerrain(b2WorldId worldId) {
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

void gencoin(float x, float y){ 
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_staticBody;
    bodyDef.position = (b2Vec2){ x , y };
    
    b2BodyId coin = b2CreateBody(worldId, &bodyDef);
    coins.push_back(coin);

    b2Circle circle;
    circle.radius = 1;
    circle.center = (b2Vec2){0.0f, 0.0f};

    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    shapeDef.friction = 0.3f;
    shapeDef.restitution = 0.2f;
    shapeDef.isSensor = true;
    // std::cout<<"coin created at: "<<x<<" , "<<y<<'\n';
    b2CreateCircleShape(coin, &shapeDef, &circle);
}

void updateTerrain(float cameraX) {
    static int cointimer = 100;
    float lastX = terrainPoints.front().x;
    // printf("lastX: %d\n",boxToScreenX(lastX));
    // printf("lastX ori: %f\n",lastX);
    // std::cout<<"firstX: "<<terrainPoints.back().x<<"\n";
    float segmentLength = TERRAIN_LENGTH / TERRAIN_SEGMENTS;
    float noiseScale = TERRAIN_WIDTH; // Same scale as generation
    float amplitude = TERRAIN_HEIGHT;



    // Generate new points
    while (lastX - cameraX < simWidth-carpos) {
        lastX += segmentLength;
        float newY = perlinNoise(lastX * noiseScale) * amplitude;
        terrainPoints.insert(terrainPoints.begin(), b2Vec2{lastX, newY});
        if(!cointimer--){
            cointimer = 100;
            gencoin(lastX,newY);
        }
    }
    while (!terrainPoints.empty() && terrainPoints.back().x < cameraX-carpos) 
        terrainPoints.pop_back();
    // printf("last xx lastX: %d\n",boxToScreenX(lastX));
    // printf("last xyx lastX ori: %f\n",lastX);
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
    b2Polygon dynamicBox = b2MakeBox(carWidth, carHeight);
    dynamicBox.centroid = (b2Vec2){0.0f, -carHeight};

    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 3.0f;
    shapeDef.friction = 0.3f;
    shapeDef.restitution = 0.2f;
    // shapeDef.

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
    wheelJointDef.hertz = 4.0f;
    wheelJointDef.dampingRatio = 0.6f;
    // wheelJointDef.enableLimit = true;
    // wheelJointDef.
    wheel1 = b2CreateWheelJoint(worldId, &wheelJointDef);

    wheelJointDef.bodyIdA = whl2;
    // wheelJointDef.bodyIdB = chasi;
    wheelJointDef.localAnchorB = (b2Vec2){carWidth-carWidth/6 , -(carHeight-carHeight/8)};

    wheel2 = b2CreateWheelJoint(worldId, &wheelJointDef);
}

void applyForwardForce(float forceMagnitude=1000, bool front=true, bool back=true, bool dir=false) {
    // Get the forward direction of the car in world coordinates
    float vecx = 0.2f,vecy = 0;
    if(front)
        vecx += 0.4f,vecy += 0.1f;
    if(back)
        vecx += 0.4f,vecy += 0.1f;
    if(dir)
        vecx = -vecx;
    b2Vec2 forwardDirection = b2Body_GetWorldVector(chasi, (b2Vec2){vecx, vecy}); // Assuming forward is local (0, -1)

    // Compute the forward force vector
    b2Vec2 forwardForce = forwardDirection * forceMagnitude;
    // printf("force dierection: %f , %f\n",forwardForce.x,forwardForce.y);
    // Apply the force at the center of mass of the car body
    b2Body_ApplyForceToCenter(chasi, forwardForce, true);
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

        // Debug output
        // std::cout << "Checking segment " << i
        //           << " | Origin: (" << rayInput.origin.x << ", " << rayInput.origin.y << ")"
        //           << " | Translation: (" << rayInput.translation.x << ", " << rayInput.translation.y << ")"
        //           << " | Hit: " << castOutput.hit
        //           << " | Fraction: " << castOutput.fraction << "\n";

        // Check for a valid hit
        if (castOutput.hit && castOutput.fraction <= 1.0f) {
            return true; // The wheel is grounded
        }
    }

    return false; // No grounding detected
}

void wheelstability(){
    float limit = 25;
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

#endif
