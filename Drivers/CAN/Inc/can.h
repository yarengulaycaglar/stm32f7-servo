/*
 * can.h
 *
 *  Created on: Mar 20, 2025
 *      Author: Yaren Gulay Caglar
 */
#ifndef CAN_INC_CAN_H_
#define CAN_INC_CAN_H_

#include <stdint.h>
#include "stm32h7xx_hal.h"

typedef enum {
	 CAN_PACKET_SET_DUTY = 0, //Duty cycle mode
	 CAN_PACKET_SET_CURRENT, //Current loop mode
	 CAN_PACKET_SET_CURRENT_BRAKE, // Current brake mode
	 CAN_PACKET_SET_RPM, //Velocity mode
	 CAN_PACKET_SET_POS, // Position mode
	 CAN_PACKET_SET_ORIGIN_HERE, //Set origin mode
	 CAN_PACKET_SET_POS_SPD, //Position velocity loop mode
} CAN_PACKET_ID;

// Global motor veri değişkenleri
extern float motor_position;
extern float motor_speed;
extern float motor_current;
extern int8_t motor_temperature;
extern int8_t motor_error;

void comm_can_set_duty(uint8_t controller_id, float duty); //Duty cycle mode

void comm_can_set_cb(uint8_t controller_id); // Current Brake Mode

void comm_can_set_rpm(uint8_t controller_id, float rpm); //Velocity mode

void comm_can_set_pos(uint8_t controller_id); // Position loop mode

void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode); //Set origin mode

void comm_can_set_current(uint8_t controller_id); //Current loop mode

void comm_can_set_pos_spd(uint8_t controller_id, int16_t RPA ); //Position and Velocity Loop Mode

void motor_receive(int8_t* rx_message); // Servo mode of motor message format

#endif /* CAN_INC_CAN_H_ */
