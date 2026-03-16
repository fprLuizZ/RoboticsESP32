/*
    Arquivo de configurações do braço
*/

#ifndef CONFIG_H
#define CONFIG_H

#define L1 160.0
#define L2 140.0
#define L3 105.0
#define SERVO_MIN 0
#define SERVO_MAX 180

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

const float OFFSETS[4] = { 90, 0, 121, 90 };
const int DIRECTIONS[4] = { 1, 1, 1, 1 };

#endif