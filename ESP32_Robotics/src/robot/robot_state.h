#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <cstdint>
#include <kinematics/kinematics.h>

struct RobotState {

    Pose pose;

    JointAngles joints; //graus

};

#endif