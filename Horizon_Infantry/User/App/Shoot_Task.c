#include "Shoot_Task.h"
#include <math.h>

#define DBUS_MID     3
#define DBUS_DOWN    1

#define FRICTION_SPEED         6300.0f

#define JAM_ERROR_THRESHOLD    0.2f
#define JAM_SPEED_THRESHOLD    30.0f
#define JAM_COUNT_THRESHOLD    50

#define MOUSE_BURST_TIME       20

Shooter_t shooter;

typedef enum
{
    SHOOT_EVENT_NONE = 0,
    SHOOT_EVENT_SINGLE,
    SHOOT_EVENT_BURST

} Shoot_Event_e;

void MOTOR_PID_Shoot_INIT(MOTOR_Typedef *motor)
{
    float PID_S_Shoot_L[3] = {15.0f, 0.0f, 0.0f};
    float PID_P_Shoot_L[3] = {0.0f, 0.0f, 0.0f};

    PID_Init(&motor->DJI_3508_Shoot_L.PID_S, 10000.0f, 0.0f, 
              PID_S_Shoot_L, 0.0f, 0.0f, 
              0.0f, 0.0f, 0, 
              Integral_Limit);
    PID_Init(&motor->DJI_3508_Shoot_L.PID_P, 1000.0f, 50.0f, 
              PID_P_Shoot_L, 0.0f, 0.0f, 
              0.0f, 0.0f, 0, 
              Integral_Limit);
    
    float PID_S_Shoot_R[3] = {15.0f, 0.0f, 0.0f};
    float PID_P_Shoot_R[3] = {0.0f, 0.0f, 0.0f};

    PID_Init(&motor->DJI_3508_Shoot_R.PID_S, 10000.0f, 0.0f, 
              PID_S_Shoot_R, 0.0f, 0.0f, 
              0.0f, 0.0f, 0, 
              Integral_Limit);
    PID_Init(&motor->DJI_3508_Shoot_R.PID_P, 1000.0f, 50.0f, 
              PID_P_Shoot_R, 0.0f, 0.0f, 
              0.0f, 0.0f, 0, 
              Integral_Limit);

    float PID_S_Shoot_G[3] = {15.0f, 0.0f, 0.0f};
    float PID_P_Shoot_G[3] = {1.0f, 0.01f, 0.0f};

    PID_Init(&motor->DJI_3508_Shoot_G.PID_S, 10000.0f, 0.0f, 
              PID_S_Shoot_G, 0.0f, 0.0f, 
              0.0f, 0.0f, 0, 
              Integral_Limit);
    PID_Init(&motor->DJI_3508_Shoot_G.PID_P, 1000.0f, 50.0f, 
              PID_P_Shoot_G, 0.0f, 0.0f, 
              0.0f, 0.0f, 0, 
              Integral_Limit);
}


static Shoot_Event_e Shooter_GetEvent(Shooter_t *s)
{
    Shoot_Event_e event = SHOOT_EVENT_NONE;

    uint8_t sw = s->dbus->Remote.S1_u8;

    if (sw == DBUS_DOWN &&
        s->ctrl.last_switch != DBUS_DOWN)
    {
        event = SHOOT_EVENT_SINGLE;
    }

    s->ctrl.last_switch = sw;

    uint8_t mouse_l = s->dbus->Mouse.L_State;

    if (mouse_l && !s->ctrl.last_mouse_l)
    {
        s->ctrl.mouse_press_cnt = 0;

        event = SHOOT_EVENT_SINGLE;
    }

    if (mouse_l)
    {
        s->ctrl.mouse_press_cnt++;

        if (s->ctrl.mouse_press_cnt > MOUSE_BURST_TIME)
        {
            event = SHOOT_EVENT_BURST;
        }
    }
    else
    {
        s->ctrl.mouse_press_cnt = 0;
    }

    s->ctrl.last_mouse_l = mouse_l;

    uint8_t mouse_r = s->dbus->Mouse.R_State;

    s->ctrl.auto_aim_enable = mouse_r;

    return event;
}

static uint8_t Shooter_IsOverHeat(User_Data_T *user)
{
#ifdef SHOOT_HEAT_ENABLE

    float heat_left =
        user->robot_status.shooter_barrel_heat_limit
      - user->power_heat_data.shooter_17mm_barrel_heat;

    return (heat_left < 40);

#else

    return 0;

#endif
}

static uint8_t Shooter_IsJam(Shooter_t *s)
{
    float err = fabsf(
        s->ctrl.target_angle
      - s->motor->DJI_3508_Shoot_G.DATA.Angle_Infinite);

    if (err > s->ctrl.single_angle * JAM_ERROR_THRESHOLD
        &&
        fabsf((float)s->motor->DJI_3508_Shoot_G.DATA.Speed_now)
        < JAM_SPEED_THRESHOLD)
    {
        s->ctrl.jam_cnt++;
    }
    else
    {
        s->ctrl.jam_cnt = 0;
    }

    return (s->ctrl.jam_cnt > JAM_COUNT_THRESHOLD);
}

static void Shooter_FireSingle(Shooter_t *s)
{
    s->ctrl.target_angle -= s->ctrl.single_angle;
}

static void Shooter_FireBurst(Shooter_t *s)
{
    s->ctrl.target_angle -= s->ctrl.single_angle;
}

static void Shooter_AutoAim(Shooter_t *s)
{
    if (s->ctrl.auto_aim_enable &&
        s->vision->target_visible)
    {
        s->ctrl.target_angle -= s->ctrl.single_angle;
    }
}

static void Shooter_Output(Shooter_t *s, MOTOR_Typedef *MOTOR)
{
    float friction = 0;

    if (!Shooter_IsOverHeat(&User_data))
    {
        friction = FRICTION_SPEED;
    }

    s->motor->DJI_3508_Shoot_L.DATA.Aim = friction;

    s->motor->DJI_3508_Shoot_R.DATA.Aim = friction;

    s->motor->DJI_3508_Shoot_G.DATA.Aim =
        s->ctrl.target_angle;

    // PID
    PID_Calculate(&MOTOR->DJI_3508_Shoot_L.PID_S, 
                   MOTOR->DJI_3508_Shoot_L.DATA.Speed_now,
                   MOTOR->DJI_3508_Shoot_L.DATA.Aim);

    PID_Calculate(&MOTOR->DJI_3508_Shoot_R.PID_S,
                   MOTOR->DJI_3508_Shoot_R.DATA.Speed_now,
                   MOTOR->DJI_3508_Shoot_R.DATA.Aim);

    PID_Calculate(&MOTOR->DJI_3508_Shoot_G.PID_P, 
                   MOTOR->DJI_3508_Shoot_G.DATA.Angle_Infinite,
                   MOTOR->DJI_3508_Shoot_G.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Shoot_G.PID_S,
                   MOTOR->DJI_3508_Shoot_G.DATA.Speed_now,
                   MOTOR->DJI_3508_Shoot_G.PID_P.Output);

    DJI_Current_Ctrl(&hcan1, 0x200, 0, 0, MOTOR->DJI_3508_Shoot_G.PID_S.Output, 0);
    DJI_Current_Ctrl(&hcan2, 0x200, 0, 0, 0, 0);
}

void Shooter_Init(
    Shooter_t *s,
    MOTOR_Typedef *motor,
    Vision_Data_t *vision)
{
    s->motor = motor;

    s->vision = vision;

    s->ctrl.single_angle = 36864.0f;

    s->ctrl.target_angle =
        s->motor->DJI_3508_Shoot_G.DATA.Angle_Infinite;

    s->ctrl.last_mouse_l = 0;

    s->ctrl.last_switch = DBUS_MID;

    s->ctrl.mouse_press_cnt = 0;

    s->ctrl.auto_aim_enable = 0;

    s->ctrl.jam_cnt = 0;

    MOTOR_PID_Shoot_INIT(motor);
}

void Shooter_Update(Shooter_t *s, DBUS_Typedef *dbus)
{
    s->dbus = dbus;

    if (Shooter_IsOverHeat(&User_data))
    {
        Shooter_Output(s, &ALL_MOTOR);
        return;
    }

    if (Shooter_IsJam(s))
    {
        s->ctrl.target_angle =
            s->motor->DJI_3508_Shoot_G.DATA.Angle_Infinite;

        Shooter_Output(s, &ALL_MOTOR);
        return;
    }

    Shoot_Event_e event =
        Shooter_GetEvent(s);

    switch (event)
    {
        case SHOOT_EVENT_SINGLE:

            Shooter_FireSingle(s);

            break;

        case SHOOT_EVENT_BURST:

            Shooter_FireBurst(s);

            break;

        default:

            break;
    }

    Shooter_AutoAim(s);

    Shooter_Output(s, &ALL_MOTOR);
}
