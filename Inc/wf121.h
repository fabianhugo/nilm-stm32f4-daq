/*
 * wf121.h
 *
 *  Created on: Jun 10, 2019
 *      Author: burr
 */


//only declarations no definitions!
#ifndef WF121_H_
#define WF121_H_

extern uint8_t rxbuffer[BGLIB_MSG_MAXLEN];

/** Length of message payload data. */
extern volatile uint16_t msg_length;

extern volatile uint8_t rxcplt;
extern volatile uint8_t awaitmore;

extern uint8_t streamingset;
extern uint8_t started_measurement;

extern uint8_t saved_endpoint;
extern uint8_t connected;
extern UART_HandleTypeDef huart1;

extern uint8_t errorcounter;
extern TIM_HandleTypeDef htim1;


void uart_output(uint8 len1,uint8* data1,uint16 len2,uint8* data2); //Implemented for sending commands according to BGAPI Manual: Bluegiga-WiFi-Software-3.0-API-RM.pdf
void wifi_init(uint8_t* rxbuffer); //Resets the module, sets up UART to wait for message
void wifi_connect(uint8_t *rxbuffer, uint8_t *streamingset); //Establishes the WiFi & TCPIP/UDP Connection. Type of Connection can be set in wf121.c by Macros.
void wifi_check(uint8_t* rxbuffer, uint8_t *streamingset); //Experimental: Checks for status updates of WiFi Module

#endif /* WF121_H_ */
