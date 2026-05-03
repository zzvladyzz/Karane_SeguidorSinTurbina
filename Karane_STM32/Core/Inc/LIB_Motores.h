/*
 * LIB_MOTORES.h
 *
 *  Created on: Apr 19, 2026
 *      Author: vlady-HP
 */

#ifndef INC_LIB_MOTORES_H_
#define INC_LIB_MOTORES_H_


#include "stm32f1xx_hal.h"
#include "stdbool.h"

typedef struct{
	int16_t PWM_MR;
	int16_t PWM_ML;
	bool ENABLE;
}Motores_Init;

void Inicializar_Motores(Motores_Init * Mot);
void Inicializar_Brushless();
void Inicializar_Motor_Coreless();
void PWM_Motores(Motores_Init * Mot);
void PWM_Brushless(uint16_t pwm);
void PWM_Motor_Coreless_Start(uint16_t vel);
void PWM_Motor_Coreless_Stop();



#endif /* INC_LIB_MOTORES_H_ */
