#include "get_K.h"
#include "can_bsp.h"
#include "All_Init.h"

boardTxData_t BoardTXData = {0};
boardRxData_t boardRxData = {0};

void Board_to_board_send(boardTxData_t *send, float yaw)
{
    send->dataNeaten.yaw_imu = yaw;
    canx_send_data(&hcan1, 0x1C, send->txData);
}

void Board_to_board_recv(boardRxData_t *recv, uint8_t *data)
{
    // memcpy(recv, data, 8);
    recv->dataNeaten.ch2 = data[1] << 8 | data[0];
    recv->dataNeaten.ch3 = data[3] << 8 | data[2];
    recv->dataNeaten.s1  = data[4];
    recv->dataNeaten.s2  = data[5];
    recv->dataNeaten.pitch = data[7] << 8 | data[6];

    WHW_V_DBUS.Remote.S1_u8 = recv->dataNeaten.s1;
    WHW_V_DBUS.Remote.S1_u8 = recv->dataNeaten.s1;
    WHW_V_DBUS.Remote.CH2_int16 = recv->dataNeaten.ch2;
    WHW_V_DBUS.Remote.CH3_int16 = recv->dataNeaten.ch3;
}