#ifndef CONTROLLER
#include "functional.h"

void applyForwardForce(float forceMagnitude=1000, bool front=true, bool back=true, int8_t dir=0) {
    // Get the forward direction of the car in world coordinates
    float vecx = 0.2f,vecy = 0;
    if(front)
        vecx += 0.4f,vecy += 0.1f;
    if(back)
        vecx += 0.4f,vecy += 0.1f;
        
    vecx *= dir;
    // std::cout<<"vecx: "<<vecx<<" , vecy: "<<vecy<<'\n';
    b2Vec2 forwardDirection = b2Body_GetWorldVector(chasi, (b2Vec2){vecx, vecy}); // Assuming forward is local (0, -1)
    b2Vec2 forwardForce = forwardDirection * forceMagnitude;

    b2Body_ApplyForceToCenter(chasi, forwardForce, true);
}

void momentun(bool whog1, bool whog2,int8_t dir){
    b2WheelJoint_SetMotorSpeed(wheel1, b2WheelJoint_GetMotorSpeed(wheel1) + 1.1f*dir);      //-ve dir is left
    b2WheelJoint_SetMotorSpeed(wheel2, b2WheelJoint_GetMotorSpeed(wheel2) + 1.1f*dir);      //+ve dir is right
    applyForwardForce(2000.0f,whog1,whog2,dir); // negative
    if(!whog1 || !whog2)
        b2Body_ApplyAngularImpulse(chasi, 10.0f*dir, true);
}

void motertogel(){
    b2WheelJoint_SetMotorSpeed(wheel1, 0);
    b2WheelJoint_SetMotorSpeed(wheel2, 0);
    b2WheelJoint_EnableMotor(wheel1, false);
    b2WheelJoint_EnableMotor(wheel2, false);
}

void over(bool &star){
    b2Vec2 speed = b2Body_GetLinearVelocity(chasi);
    // std::cout<<"speed: "<<speed.x<<" , "<<speed.y<<'\r';
    static int ticks = 100;
    if(abs(speed.x) < 0.1 && abs(speed.y) < 0.1){
        // std::cout<<"tick: "<<ticks<<"    \r"; 
        if (!(ticks--)){
            gameover = true;
            star = false;
            std::cout<<"game over       \n";
            ticks = 100;
        }
    }
    else
        ticks = 100;
}

#endif