#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include <config.h>
#include <math.h>



struct Pose {
    float x;
    float y;
    float z;
    float phi; //phi em graus
};

struct JointAngles
{
    float t0;
    float t1;
    float t2;
    float t3;
};

struct Solution {
    bool exist;
    JointAngles thetas_deg;
};

class Kinematics {

    public:

        static Pose forward(JointAngles j);

        static Solution inverse(Pose p, JointAngles &j);
    
        static float thetaToServo(int joint, float theta_deg);

};


#endif
