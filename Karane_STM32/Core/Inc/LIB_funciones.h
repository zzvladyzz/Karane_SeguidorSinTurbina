/*
 * LIB_funciones.h
 *
 *  Created on: Dec 22, 2025
 *      Author: Vlady-Chuwi
 */

#ifndef INC_LIB_FUNCIONES_H_
#define INC_LIB_FUNCIONES_H_
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
/**
 * @brief 	Constantes fisicas del robot para odometria
 * @note	Calcular valores para tener un resultado optimo
 * @note	PODEMOS USAR LOS TICK DEL ENCODER COMO AJUSTE PARA CIERTO PARAMETROS ASI EVITAMOS EL USO DE
 * 			LA BOTONERA
 */
#define TICKS_POR_REVOLUCION  200.0 // Cuantos ticks da el encoder por vuelta completa
#define DIAMETRO_RUEDA_MM      	29.1571 // Diámetro de la rueda en mm y grosor 12,7
#define DISTANCIA_ENTRE_RUEDAS_MM 105.8 // Distancia entre los centros de las ruedas
#define LONGITUD_MUESTRA_MM (M_PI * DIAMETRO_RUEDA_MM / TICKS_POR_REVOLUCION)
/**
 * @brief Variables de estado global del robot para odometria
 */
typedef struct{
	float robot_X;     // Posición X actual en mm
	float robot_Y;     // Posición Y actual en mm
	float robot_Angulo_rad; // Orientación/Rumbo actual en radianes
	float robot_Angulo_deg;
	volatile int32_t ticks_L ; // Contadores de los encoders (desde el callback EXTI)
	volatile int32_t ticks_R ;
}odometria_init_t;

/**
 * @brief Estructura para el PID
 */
typedef struct
{
	float	kp;
	float	ki;
	float	kd;
	float	ultimoError;
	float	integral;
	float	winup;
	float	PID_Maximo;

}PID;

void funcion_odometria(odometria_init_t* odometria);
float funcion_calcularPID(PID* pid,float setpoint,float actual,float dt);
float funcion_Filtro_Kalman_odometria(float delta_theta_encoders,float gyro_rate_z,float dt);

#endif /* INC_LIB_FUNCIONES_H_ */
