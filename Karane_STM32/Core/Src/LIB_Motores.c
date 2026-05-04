#include <LIB_Motores.h>
#include "tim.h"
#include "gpio.h"


/*
 * Definicion para ventilador ya sea Brushless o Coreless
 */
#define PWM_Timer_BRUSHLESS	htim3
#define PWM_BRUSHLESS	TIM_CHANNEL_4
#define PWM_Brushless_MAX	2000
#define PWM_Brushless_MIN	1000

/*
 * Definiciones para Motores Coreless como succion
 */
#define PWM_Timer_Coreless	htim3
#define PWM_Coreless		TIM_CHANNEL_4
#define PWM_Coreless_Max	999


/*
 * Definiciones para los motores
 */
#define PWM_Timer_Mot	htim1
#define PWM_ML_A		TIM_CHANNEL_1
#define PWM_ML_B		TIM_CHANNEL_2

#define PWM_MR_B		TIM_CHANNEL_3
#define PWM_MR_A		TIM_CHANNEL_4

#define Enable_PORT		GPIOA
#define Enable_PIN		GPIO_PIN_12

#define PWM_MAX			999



void Inicializar_Motores(Motores_Init * Mot)
{
	HAL_GPIO_WritePin(Enable_PORT, Enable_PIN, GPIO_PIN_RESET);
	HAL_TIM_PWM_Start(&PWM_Timer_Mot, PWM_ML_A);
	HAL_TIM_PWM_Start(&PWM_Timer_Mot, PWM_ML_B);
	HAL_TIM_PWM_Start(&PWM_Timer_Mot, PWM_MR_B);
	HAL_TIM_PWM_Start(&PWM_Timer_Mot, PWM_MR_A);

	__HAL_TIM_SET_COMPARE(&PWM_Timer_Mot,PWM_ML_A,0);
	__HAL_TIM_SET_COMPARE(&PWM_Timer_Mot,PWM_ML_B,0);
	__HAL_TIM_SET_COMPARE(&PWM_Timer_Mot,PWM_MR_B,0);
	__HAL_TIM_SET_COMPARE(&PWM_Timer_Mot,PWM_MR_A,0);
	Mot->ENABLE=false;
	Mot->PWM_ML=0;
	Mot->PWM_MR=0;

}
void Inicializar_Brushless()
{

	HAL_TIM_PWM_Start(&PWM_Timer_BRUSHLESS, PWM_BRUSHLESS);
	__HAL_TIM_SET_COMPARE(&PWM_Timer_BRUSHLESS	,PWM_BRUSHLESS,1000);
	HAL_Delay(3000);
}
void Inicializar_Motor_Coreless()
{
	HAL_TIM_PWM_Stop(&PWM_Timer_Coreless	, PWM_Coreless);
	__HAL_TIM_SET_COMPARE(&PWM_Timer_BRUSHLESS,PWM_Coreless,0);
}

void PWM_Motor_Coreless_Start(uint16_t vel)
{
	vel=(vel>PWM_Coreless_Max)?999:vel;
	HAL_TIM_PWM_Start(&PWM_Timer_Coreless, PWM_Coreless);

	for(uint16_t x=0; x<vel;x+=10)
	{
		__HAL_TIM_SET_COMPARE(&PWM_Timer_Coreless,PWM_Coreless,x);
		HAL_Delay(500);
	}
}
void PWM_Motor_Coreless_Stop()
{
	HAL_TIM_PWM_Stop(&PWM_Timer_Coreless, PWM_Coreless);
}

void PWM_Brushless(uint16_t pwm)
{
	pwm=(pwm>2000)?2000:pwm;
	pwm=(pwm<1000)?1000:pwm;
	__HAL_TIM_SET_COMPARE(&PWM_Timer_BRUSHLESS,PWM_BRUSHLESS,pwm);
}
void PWM_Motores(Motores_Init * Mot)
{
	HAL_GPIO_WritePin(Enable_PORT, Enable_PIN, Mot->ENABLE);
	int16_t PWM_ML=0;

	if((Mot->PWM_ML)<0)
	{
		Mot->PWM_ML=((Mot->PWM_ML)<-PWM_MAX)?-PWM_MAX:Mot->PWM_ML;

		PWM_ML=999+Mot->PWM_ML;
		__HAL_TIM_SET_COMPARE(&PWM_Timer_Mot,PWM_ML_A,999);
		__HAL_TIM_SET_COMPARE(&PWM_Timer_Mot,PWM_ML_B,PWM_ML);

	}
	else
	{
		Mot->PWM_ML=((Mot->PWM_ML)>PWM_MAX)?PWM_MAX:Mot->PWM_ML;

		PWM_ML=999-Mot->PWM_ML;
		__HAL_TIM_SET_COMPARE(&PWM_Timer_Mot,PWM_ML_A,PWM_ML);
		__HAL_TIM_SET_COMPARE(&PWM_Timer_Mot,PWM_ML_B,999);
	}

	int16_t PWM_MR=0;

	if((Mot->PWM_MR)<0)
	{
		Mot->PWM_MR=((Mot->PWM_MR)<-PWM_MAX)?-PWM_MAX:Mot->PWM_MR;

		PWM_MR=999+Mot->PWM_MR;
		__HAL_TIM_SET_COMPARE(&PWM_Timer_Mot,PWM_MR_A,999);
		__HAL_TIM_SET_COMPARE(&PWM_Timer_Mot,PWM_MR_B,PWM_MR);
	}
	else
	{
		Mot->PWM_MR=((Mot->PWM_MR)>PWM_MAX)?PWM_MAX:Mot->PWM_MR;

		PWM_MR=999-Mot->PWM_MR;
		__HAL_TIM_SET_COMPARE(&PWM_Timer_Mot,PWM_MR_A,PWM_MR);
		__HAL_TIM_SET_COMPARE(&PWM_Timer_Mot,PWM_MR_B,999);
	}

}


