/*
 * DHT22.h
 *
 *  Created on: Feb 20, 2020
 *      Author: mrhause
 */
#include "stm32f1xx_hal.h"

#ifndef INC_DHT22_H_
#define INC_DHT22_H_
//DEFINE DHT_PORT and DHT_PIN to which sensor is conected
#define DHT_PIN DHT22_Pin
#define DHT_PORT DHT22_GPIO_Port

void DHT22_Init(GPIO_TypeDef* Port, uint16_t Pin); //initialization function
void ONE_WIRE_Start(); //Start communication
void ONE_WIRE_Response(); //Sensor response for start command
void gpio_output(); //set gpio as output
void gpio_input(); //set gpio as input
static void delay_my(uint32_t uSecDelay); //microsecond delay required for 1wire communication
uint8_t ONE_WIRE_Read_Data(); //read data from sensor
uint8_t DHT22_Get_Data(uint16_t *Temp,uint16_t *Hum); //function available for user to use

#endif /* INC_DHT22_H_ */


