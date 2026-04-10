/*
 * LIB_DEBUG.c
 *
 *  Created on: Dec 22, 2025
 *      Author: Vlady-Chuwi
 */

#include "LIB_DEBUG.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define UART	huart3
extern UART_HandleTypeDef UART;

#ifndef NDEBUG

static char bufferTxt[30];//buffer para enviar datos maximo 30

void DEBUG_Imprimir(char* texto)
{
	if (texto==NULL) return;
	uint16_t size=(uint16_t)strlen(texto);
	HAL_UART_Transmit(&UART, (uint8_t *)texto, size, HAL_MAX_DELAY);
}
void DEBUG_Encoders(int32_t encL, int32_t encR,float longitud)
{

	sprintf(bufferTxt," EncL=%ld ",encL);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," EncR=%ld " ,encR);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," LEncL=%0.2f ",encL*longitud);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," LEncR=%0.2f \r \n" ,encR*longitud);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
}
void DEBUG_Odometria(float angulo,float x,float y)
{
	sprintf(bufferTxt," angulo=%0.2f ",angulo);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," x=%0.2f ",x);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," y=%0.2f \r \n",y);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
}
void DEBUG_IMU_Conv(float ax,float ay,float az,float gx,float gy,float gz )
{
	sprintf(bufferTxt," AX=%0.2f ",ax);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," AY=%0.2f ",ay);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," AZ=%0.2f ",az);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," GX=%0.2f ",gx);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," GY=%0.2f ",gy);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," GZ=%0.2f \r \n",gz);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
}
void DEBUG_PID1()
{

}
void DEBUG_PID2()
{

}
void DEBUG_Menu()
{

}

void DEBUG_ADC_RAW(uint16_t A0,uint16_t A1,uint16_t A2,uint16_t A3)
{
	sprintf(bufferTxt," A0=%u ",A0);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," A1=%u ",A1);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," A2=%u ",A2);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," A3=%u \r\n ",A3);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
}
void DEBUG_ADC_Value(float A0,float A1,float A2,float A3)
{
	sprintf(bufferTxt," A_MD=%0.2f ",A0);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," A_MI=%0.2f ",A1);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," BATT=%0.2f ",A2);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," Selector=%0.2f \r\n ",A3);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
}
void DEBUG_RegletaSensores(uint16_t S0)
{
	sprintf(bufferTxt," Posicion =%u \r\n",S0);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
}
#endif

