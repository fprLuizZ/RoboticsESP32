#include <control/motion_controller.h>

void MotionController::begin() {

    _driver.begin();

    _state.joints = {0, 90, -45, -45};

}

bool MotionController::moveTo(Pose target) {

    Solution solution = Kinematics::inverse(target, _state.joints);

    if (solution.exist) {

        JointAngles servos_deg;

        servos_deg.t0 = Kinematics::thetaToServo(0, solution.thetas_deg.t0);
        servos_deg.t1 = Kinematics::thetaToServo(1, solution.thetas_deg.t1);
        servos_deg.t2 = Kinematics::thetaToServo(2, solution.thetas_deg.t2);
        servos_deg.t3 = Kinematics::thetaToServo(3, solution.thetas_deg.t3);

        if (
        servos_deg.t0 < SERVO_MIN || servos_deg.t0 > SERVO_MAX ||
        servos_deg.t1 < SERVO_MIN || servos_deg.t1 > SERVO_MAX ||
        servos_deg.t2 < SERVO_MIN || servos_deg.t2 > SERVO_MAX ||
        servos_deg.t3 < SERVO_MIN || servos_deg.t3 > SERVO_MAX 
    
        )
        {
            Serial.println("Erro: ângulo resulta em servo fora dos limites");
            return false;
        }

        _state.joints = servos_deg;

        _driver.setAngle(0, _state.joints.t0);
        _driver.setAngle(1, _state.joints.t1);
        _driver.setAngle(2, _state.joints.t2);
        _driver.setAngle(3, _state.joints.t3);

    }

}

Pose MotionController::getPose() {

    return Kinematics::forward(_state.joints);

}