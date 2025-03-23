#include "../Inc/can.h"
#include <stdio.h>

static void comm_can_transmit_eid(uint32_t id, const uint8_t data, uint8_t len);
void Error_Handler(void);

float motor_position = 0.0f;
float motor_speed = 0.0f;
float motor_current = 0.0f;
int8_t motor_temperature = 0;
int8_t motor_error = 0;

static void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = (number >> 24) & 0xFF;
	buffer[(*index)++] = (number >> 16) & 0xFF;
	buffer[(*index)++] = (number >> 8)  & 0xFF;
	buffer[(*index)++] = number & 0xFF;
}

static void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
	buffer[(*index)++] = (number >> 8) & 0xFF;
	buffer[(*index)++] = number & 0xFF;
}

// Duty cycle mode
void comm_can_set_duty(uint8_t controller_id, float duty) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0f), &send_index);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

// Current Brake Mode
void comm_can_set_cb(uint8_t controller_id) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(motor_current * 1000.0f), &send_index);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

// Velocity mode
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
	printf("RPM is setting...\n");
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

// Position loop mode
void comm_can_set_pos(uint8_t controller_id) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(motor_position * 1000000.0f), &send_index);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

// Set origin mode
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
	uint8_t buffer[1];
	buffer[0] = set_origin_mode;
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8), buffer, 1);
}

// Current loop mode
void comm_can_set_current(uint8_t controller_id) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(motor_current * 1000.0f), &send_index);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

// Position and Velocity Loop Mode
void comm_can_set_pos_spd(uint8_t controller_id, int16_t RPA) {
	int32_t send_index = 0;
	int16_t send_index1 = 0;
	uint8_t buffer[8] = {0};
	buffer_append_int32(buffer, (int32_t)(motor_position * 10000.0f), &send_index);
	buffer_append_int16(buffer + send_index, motor_speed, &send_index1);
	buffer_append_int16(buffer + send_index + send_index1, RPA, &send_index1);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index + send_index1);
}

// Transmit fonksiyonu (FDCAN kullanılarak)
static void comm_can_transmit_eid(uint32_t id, const uint8_t data, uint8_t len) {
	if (len > 8) {
		len = 8;
	}
	FDCAN_TxHeaderTypeDef TxHeader;
	TxHeader.Identifier = id;
	TxHeader.IdType = FDCAN_STANDARD_ID; // Standart ID tipi
	TxHeader.TxFrameType = FDCAN_DATA_FRAME; // Veri çerçevesi
	TxHeader.DataLength = len;           // Veri uzunluğu
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;

	FDCAN_HandleTypeDef hfdcan1;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, (uint8_t*)data) != HAL_OK)
	{
		Error_Handler(); // Veri gönderme hatası
	}
}

// motor_receive: CAN Rx mesajındaki veriyi işleyip global değişkenlere atar.
void motor_receive(int8_t* rx_message)
{
	int16_t pos_int = rx_message[0] << 8 | rx_message[1];
	int16_t spd_int = rx_message[2] << 8 | rx_message[3];
	int16_t cur_int = rx_message[4] << 8 | rx_message[5];

	motor_position= (float)( pos_int * 0.1f); // Motor Position
	motor_speed= (float)( spd_int * 10.0f);// Motor Speed
	motor_current= (float) ( cur_int * 0.01f);// Motor Current
	motor_temperature= rx_message[6] ;// Motor Temperature
	motor_error= rx_message[7] ;// Motor Error Code
	printf("Recieving data:\n motor_position: %f\n motor_speed: %f\nmotor_current: %f\nmotor_temperature: %d\nmotor_error: %d\n",
			motor_position, motor_speed, motor_current, motor_temperature, motor_error);
}
