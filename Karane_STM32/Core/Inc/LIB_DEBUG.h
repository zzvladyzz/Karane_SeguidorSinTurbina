/*
 * LIB_DEBUG.h
 *
 *  Created on: Dec 22, 2025
 *      Author: Vlady-Chuwi
 */

#ifndef INC_LIB_DEBUG_H_
#define INC_LIB_DEBUG_H_
#include <stdint.h>

/*
 * Al usar #define NDEBUG no se usara nada de esta libreria
 * donde crearemos los datos que mandaremos al uart
 */
#ifndef NDEBUG

	void DEBUG_Encoders(int32_t encL, int32_t encR,float longitud);
	void DEBUG_Odometria(float angulo,float x,float y);
	void DEBUG_IMU_Conv(float ax,float ay,float az,float gx,float gy,float gz);
	void DEBUG_PID1();
	void DEBUG_PID2();
	void DEBUG_Menu();
	void DEBUG_ADC_RAW(uint16_t A0,uint16_t A1,uint16_t A2,uint16_t A3);
	void DEBUG_ADC_Value(float A0,float A1,float A2,float A3);
	void DEBUG_RegletaSensores(uint16_t S0);
	void DEBUG_Imprimir(char* texto);

#else
	#define DEBUG_Encoders(a,b,c)
	#define DEBUG_Odometria(a,x,y)
	#define DEBUG_IMU_Conv(a,b,c,d,e,f)
	#define DEBUG_PID1()
	#define DEBUG_PID2()
	#define DEBUG_Menu()
	#define DEBUG_Imprimir(c)
	#define DEBUG_ADC_RAW(a,b,c,d)
	#define DEBUG_ADC_Value(a,b,c,d)
	#define DEBUG_RegletaSensores(s0)
#endif


#endif /* INC_LIB_DEBUG_H_ */
