#ifndef CONTROLLER
#include "functional.h"

void fillcoinvals(){
    int valslen = 1000;
    coinvals = new int[valslen];
    for(int i=0;i<valslen;i++){
        if(i<50)
            coinvals[i] = 500;
        else if(i<150)
            coinvals[i] = 250;
        else if(i<250)
            coinvals[i] = 180;
        else if(i<450)
            coinvals[i] = 120;
        else if(i<500)
            coinvals[i] = 90;
        else if(i<600)
            coinvals[i] = 50;
        else if(i<700)
            coinvals[i] = 40;
        else if(i<800)
            coinvals[i] = 30;
        else if(i<900)
            coinvals[i] = 20;
        else
            coinvals[i] = 10;

    }
        // coinvals[i] = 1;
}

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