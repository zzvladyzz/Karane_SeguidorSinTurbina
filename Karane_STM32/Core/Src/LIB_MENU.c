/*
 * LIB_LED_INDA.c
 *
 *  Created on: Dec 13, 2025
 *      Author: Vlady-Chuwi
 */
#include "LIB_MENU.h"
#include "gpio.h"


Indicador_LED_e	indicadorLED=Apagar_LED;
Menu_Parametros_e 	Menu_Global=Menu_Inicial;
static bool BTN_SEL=false;
static Tiempo_espera_s Tiempo={0,true,0};


/**
 *	@brief 	Se ejecuta un tiempo de espera para los avisos
 */
void	Tiempo_Espera(void)
{

	uint32_t tiempo_actual=HAL_GetTick();
	switch (Tiempo.estado_interno) {
		case 0:
			Tiempo.tiempo_de_espera=tiempo_actual;
			Tiempo.espera=true;
			Tiempo.estado_interno=1;
			break;
		case 1:
			if(tiempo_actual-Tiempo.tiempo_de_espera>300)
			{
				Tiempo.estado_interno=2;
			}
			break;
		case 2:
			Tiempo.estado_interno=0;
			Tiempo.espera=false;
			break;
		default:
			break;
	}
}

/**
 * @brief 	Menu donde se vera en que opcion entrar
 * @param	MENU: Se debe inicializar el ENUM antes para poder mover entre menus
 * @param 	BTN: Es una valor de 1 a 3 que viene de los pulsadores ya filtrado
 * @return	retornara un valor del menu en el que estemos
 * @note	Una vez terminada esta funcion verificar que el valor del adc o del pulsador quede
 * 			en cero para evitar los rebotes innecesarios
 */
uint8_t 	Menu_Navegacion(uint8_t BTN)
{

	if(BTN_SEL==false){
		switch (BTN) {
			case BTN_DERECHA:
				Menu_Global=(Menu_Global==Menu_Inicial)?Menu_Final:(Menu_Global-1);
				Menu_aviso(Estado_BTN_derecha);
				Menu_ubicacion(Menu_Global);
				break;
			case BTN_IZQUIERDA:
				Menu_Global=(Menu_Global==Menu_Final)?Menu_Inicial:(Menu_Global+1);
				Menu_aviso(Estado_BTN_izquierda);
				Menu_ubicacion(Menu_Global);
				break;
			case BTN_ACEPTAR:
				Menu_Global=Menu_Global;
				BTN_SEL=true;
				Menu_aviso(Estado_BTN_ok);
				Menu_ubicacion(Menu_Global);
				break;
			default:
				break;
		}
	}
	else{
		switch (BTN) {
			case BTN_ACEPTAR:
				BTN_SEL=false;
				Menu_aviso(Estado_BTN_salir);
				Menu_ubicacion(Menu_Global);
				break;
			default:
				break;
		}
	}

	return Menu_Global;
}

/**
 * @brief 	Dependiendo el menu seleccionado se activara unos leds indicando el menu donde esta
 * @return	Devolvera un valor uint8
 */
bool	Menu_Ejecucion(void)
{
	if(BTN_SEL==true){
		return true;
	}
	return false;
}

/**
 * @brief 	Aca se indicara mediante leds los errores o avisos
 * @param 	LED: Se debera mandar una estructura para saber que aviso atender con los LED's
 * @note	Esta funcion necesita ser llamada mediante los retardos de esta misma lib para parpadear
 * 			los avisos
 */
void	Menu_aviso(Indicador_LED_e LED)
{
	switch(LED){
		case Estado_BTN_derecha:
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
			break;
		case Estado_BTN_izquierda:
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
			break;
		case Estado_BTN_ok:
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
			break;
		case Estado_BTN_salir:
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
			break;
		case Apagar_LED:
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
			break;

		case Aviso_bateria_baja:
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
			break;
		case Aviso_ok:
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
			break;
		case Aviso_fallo:
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
			break;
		default:
			break;

		}
}


/**
 * @brief 	Aca se indicara mediante leds el menu actual
 * @param 	menu: Se debera mandar una estructura del menu de navegacion
 * 			para saber en que menu se esta
 * @note	Esta funcion puede variar
 */
void	Menu_ubicacion(Menu_Parametros_e menu){
	uint8_t valorMenu=menu;

			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, !((valorMenu&8)>>3));
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, !((valorMenu&4)>>2));
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, !((valorMenu&2)>>1));
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, !((valorMenu&1)));
}
