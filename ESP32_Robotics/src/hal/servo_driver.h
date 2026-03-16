#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <cstdint>


class ServoDriver {

public:

    void begin();

    void setAngle(uint8_t id, float angle);

};

#endif