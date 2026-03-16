#pragma once

#include <hal/servo_driver.h>
#include <kinematics/kinematics.h>
#include <robot/robot_state.h>

class MotionController {

public:

    void begin();
    bool moveTo(Pose target);
    Pose getPose();

private:

    ServoDriver _driver;
    RobotState _state;

};