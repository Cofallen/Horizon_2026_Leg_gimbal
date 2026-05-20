#pragma once

#include "main.h"
#include "All_Init.h"

typedef struct
{
    uint8_t target_visible;

} Vision_Data_t;

typedef struct
{
    float single_angle;

    float target_angle;

    uint8_t last_mouse_l;

    uint8_t last_switch;

    uint8_t auto_aim_enable;

    uint16_t mouse_press_cnt;

    uint16_t jam_cnt;

} Shooter_Control_t;

typedef struct
{
    Shooter_Control_t ctrl;

    MOTOR_Typedef *motor;

    DBUS_Typedef *dbus;

    Vision_Data_t *vision;

} Shooter_t;

void Shooter_Init(
    Shooter_t *s,
    MOTOR_Typedef *motor,
    Vision_Data_t *vision);

void Shooter_Update(
    Shooter_t *s,
    DBUS_Typedef *dbus);
