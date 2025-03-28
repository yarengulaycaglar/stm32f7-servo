#include "../Inc/can.h"
#include <stdio.h>

static void comm_can_transmit_eid(uint32_t id, uint8_t* data, uint8_t len, FDCAN_HandleTypeDef *hfdcan1);
void Error_Handler(void);
static float uint_to_float(int x_int, float x_min, float x_max, int bits);
static int float_to_uint(float x, float x_min, float x_max, unsigned int bits);

float motor_position = 0.0f;
float motor_speed = 0.0f;
float motor_current = 0.0f;
int8_t motor_temperature = 0;
int8_t motor_error = 0;

/// limit data to be within bounds ///
float P_MIN =-95.5;
float P_MAX =95.5;
float V_MIN =-30;
float V_MAX =30;
float T_MIN =-18;
float T_MAX =18;
float KP_MIN =0;
float KP_MAX =500;
float KD_MIN =0;
float KD_MAX =5;
float Test_Pos=0.0;

uint8_t msg[12] = {0};

float p_des=0;
float v_des=0;
float kp=0;
float kd=0;
float t_ff=0;

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
void comm_can_set_duty(uint8_t controller_id, float duty, FDCAN_HandleTypeDef *hfdcan1) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0f), &send_index);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index, hfdcan1);
}

// Current Brake Mode
void comm_can_set_cb(uint8_t controller_id, FDCAN_HandleTypeDef *hfdcan1) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(motor_current * 1000.0f), &send_index);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index, hfdcan1);
}

// Velocity mode
void comm_can_set_rpm(uint8_t controller_id, float rpm, FDCAN_HandleTypeDef *hfdcan1) {
	printf("RPM is setting...\n");
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index, hfdcan1);
}

// Position loop mode
void comm_can_set_pos(uint8_t controller_id, FDCAN_HandleTypeDef *hfdcan1) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(motor_position * 1000000.0f), &send_index);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index, hfdcan1);
}

// Set origin mode
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode, FDCAN_HandleTypeDef *hfdcan1) {
	uint8_t buffer[1];
	buffer[0] = set_origin_mode;
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8), buffer, 1, hfdcan1);
}

// Current loop mode
void comm_can_set_current(uint8_t controller_id, FDCAN_HandleTypeDef *hfdcan1) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(motor_current * 1000.0f), &send_index);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index, hfdcan1);
}

// Position and Velocity Loop Mode
void comm_can_set_pos_spd(uint8_t controller_id, int16_t RPA, FDCAN_HandleTypeDef *hfdcan1) {
	int32_t send_index = 0;
	int16_t send_index1 = 0;
	uint8_t buffer[8] = {0};
	buffer_append_int32(buffer, (int32_t)(motor_position * 10000.0f), &send_index);
	buffer_append_int16(buffer + send_index, motor_speed, &send_index1);
	buffer_append_int16(buffer + send_index + send_index1, RPA, &send_index1);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index + send_index1, hfdcan1);
}

// Transmit fonksiyonu (FDCAN kullanılarak)
static void comm_can_transmit_eid(uint32_t id, uint8_t* data, uint8_t len, FDCAN_HandleTypeDef *hfdcan1) {
	if (len > 8) {
		len = 8;
	}
	FDCAN_TxHeaderTypeDef TxHeader;
	TxHeader.Identifier = id;
	TxHeader.IdType = FDCAN_STANDARD_ID; // Standart ID tipi
	TxHeader.TxFrameType = FDCAN_DATA_FRAME; // Veri çerçevesi
	TxHeader.DataLength = (len << 16); // Veri uzunluğu
	TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	// FIFO dolu mu kontrolü
    	if ((hfdcan1->Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U) {
        	return HAL_BUSY; // FIFO dolu, gönderim yapılmadı
    	}

	if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, &TxHeader, data) != HAL_OK)
	{
		Error_Handler(); // Veri gönderme hatası
	}
}

// motor_receive: CAN Rx mesajındaki veriyi işleyip global değişkenlere atar.
void motor_receive(uint8_t* rx_message)
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


/*
 ****************************************************************************************************************
 * MIT MODE
 *****************************************************************************************************************
 */

void enter_motor_control_mode(uint8_t controller_id, FDCAN_HandleTypeDef *hfdcan1) {
    uint8_t buffer[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}; // Motor control mode giriş mesajı
    comm_can_transmit_eid(controller_id, buffer, sizeof(buffer), hfdcan1);
}


static int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
	/// Converts a float to an unsigned int, given range and number of bits ///
	float span = x_max- x_min;
	if(x < x_min) x = x_min;
	else if(x > x_max) x = x_max;
	return (int) ((x- x_min)*((float)((1<<bits)/span)));
}


/*
 *  Sends routine code
 */
void pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff, FDCAN_HandleTypeDef *hfdcan1){
	p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
	v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
	kp =fminf(fmaxf(KP_MIN, kp), KP_MAX);
	kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
	t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);

	/// convert floats to unsigned ints ///
	int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
	int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
	int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
	int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
	int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

	/// pack ints into the can buffer ///
	msg[0] = p_int>>8; // Position 8 higher
	msg[1] = p_int&0xFF;// Position 8 lower
	msg[2] = v_int>>4; // Speed 8 higher
	msg[3] = ((v_int&0xF)<<4)|(kp_int>>8); //Speed 4 bit lower KP 4bit higher
	msg[4] = kp_int&0xFF; // KP 8 bit lower
	msg[5] = kd_int>>4; // Kd 8 bit higher
	msg[6] = ((kd_int&0xF)<<4)|(kp_int>>8); //KP 4bit lower torque 4 bit higher
	msg[7] = t_int&0xff; // torque 4 bit lower
	comm_can_transmit_eid(0, msg, sizeof(msg), hfdcan1);
}


/*
 *  Receive routine code
 */
void unpack_reply(){
	/// unpack ints from can buffer ///
	int id = msg[0];
	int p_int = (msg[1]<<8)|msg[2]; //Motor position data
	int v_int = (msg[3]<<4)|(msg[4]>>4); // Motor speed data
	int i_int = ((msg[4]&0xF)<<8)|msg[5]; // Motor torque data

	/// convert ints to floats ///
	float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
	float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
	float i = uint_to_float(i_int, T_MIN, T_MAX, 12);

	// Read the corresponding data according to the ID code
	if(id == 1){
		p_des = p;
		v_des = v;
		t_ff = i;
	}
}


/*
 *  All numbers are converted to floating-point by the following function
 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
