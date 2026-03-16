#include "servo_driver.h"
#include <ESP32Servo.h>

static Servo servos[5];

void ServoDriver::begin() {

    servos[0].attach(16); //Base
    servos[1].attach(17); //Ombro
    servos[2].attach(18); //Cotovelo
    servos[3].attach(19); //Punho
    servos[4].attach(4); //ferramenta

}

void ServoDriver::setAngle(uint8_t id, float angle) {

    if(id < 5)
        servos[id].write(angle);

}