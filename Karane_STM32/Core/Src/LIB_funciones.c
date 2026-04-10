/*
 * LIB_funciones.c
 *
 *  Created on: Dec 22, 2025
 *      Author: Vlady-Chuwi
 */
#include "LIB_funciones.h"
#include "tim.h"
#include "gpio.h"
#include <stdint.h>
#define Motor htim1

extern TIM_HandleTypeDef Motor;

/**
 * @brief Codigo para odometria implementarlo en un timer para la ejecucion
 * @param odometria se inicializa con los datos a usar
 */
 void funcion_odometria(odometria_init_t* odometria)
{
	/*
	* 1. Obtener los ticks desde la última vez y resetear contadores para evitar overflow de datos
	* Usamos variables intermedias para ser thread-safe (si las interrupciones EXTI pueden ocurrir ahora)
	*/
		int32_t delta_ticks_L = odometria->ticks_L;
		int32_t delta_ticks_R = odometria->ticks_R;
		odometria->ticks_L = 0; // Resetear el contador
		odometria->ticks_R = 0; // Resetear el contador

	/* 2. Convertir ticks a distancia lineal recorrida (en mm) */
		float distancia_L = delta_ticks_L * LONGITUD_MUESTRA_MM;
		float distancia_R = delta_ticks_R * LONGITUD_MUESTRA_MM;

	/* Distancia promedio recorrida por el centro del robot */
		float delta_distancia = (distancia_L + distancia_R) / 2.0;

	/* 3. Calcular el cambio en la orientación angular (en radianes) usado para filtro kalman con gyroZ */
		float delta_angulo = (distancia_R - distancia_L) / DISTANCIA_ENTRE_RUEDAS_MM;

	/**
	 * 4. Actualizar la posición global del robot (Integración)
	 * Usamos el ángulo actual + la mitad del cambio angular para mayor precisión (Runge-Kutta de 2do orden)
	 */
		float angulo_promedio = odometria->robot_Angulo_rad + delta_angulo / 2.0;

	/* Calcular el desplazamiento en X e Y y sumarlo a la posición actual */
		odometria->robot_X += delta_distancia * cos(angulo_promedio);
		odometria->robot_Y += delta_distancia * sin(angulo_promedio);

	/* 5. Actualizar el ángulo global del robot */
		odometria->robot_Angulo_rad += delta_angulo;

	/* Opcional: Mantener el ángulo entre -PI y PI (para evitar overflow float) */
		if (odometria->robot_Angulo_rad > M_PI) odometria->robot_Angulo_rad -= 2 * M_PI;
		if (odometria->robot_Angulo_rad < -M_PI) odometria->robot_Angulo_rad += 2 * M_PI;
		odometria->robot_Angulo_deg=(odometria->robot_Angulo_rad*180.0)/M_PI;
}
 /**
  * @brief Fusiona encoders y giroscopio para obtener Theta
  * @param delta_theta_encoders: El cambio de ángulo calculado por encoders
  * @param gyro_rate_z: Velocidad angular del eje Z (en rad/s) del sensor IMU
  * @param dt: Tiempo transcurrido (ej. 0.01f para 10ms)
  * @return Angulo fusionado y filtrado
  */
float funcion_Filtro_Kalman_odometria(float delta_theta_encoders,float gyro_rate_z,float dt)
 {
	/* --- Variables del Filtro de Kalman --- */
	float Q_angle = 0.001f;   // Proceso: Confianza en el modelo de los encoders
	float R_angle = 0.01f;    // Medición: Confianza en el Giroscopio
	float angle_estimado = 0.0f;
	float P_error = 1.0f;     // Covarianza del error
	float K_gain = 0.0f;      // Ganancia de Kalman

	/*
	 * --- PASO 1: PREDICCIÓN (Basada en encoders) ---
	 * Usamos el movimiento de los encoders como base del modelo
	 */
		angle_estimado += delta_theta_encoders;
		P_error += Q_angle;
	/*
	 * --- PASO 2: MEDICIÓN (Basada en Giroscopio) ---
	 * El giroscopio mide velocidad, así que lo integramos para tener ángulo
	 */
		float angulo_medido_gyro = angle_estimado + (gyro_rate_z * dt);

	/*
	 * --- PASO 3: ACTUALIZACIÓN (Fusión) ---
	 * Calcular la Ganancia de Kalman
	 */
		K_gain = P_error / (P_error + R_angle);

	/* Corregir el ángulo estimado con la medición del gyro */
		angle_estimado = angle_estimado + K_gain * (angulo_medido_gyro - angle_estimado);

	/* Actualizar la covarianza del error para el próximo ciclo */
		P_error = (1.0f - K_gain) * P_error;

		return angle_estimado;
 }

/**
 * @brief Control de los motores del robot
 * @param motores: se debe mandar una estructura con los datos a colocar
 */
void funcion_motores(motores_init_t* motores)
{
	motores->pwmLA=(motores->pwmLA>999)?999:motores->pwmLA;
	motores->pwmLB=(motores->pwmLB>999)?999:motores->pwmLB;
	motores->pwmRA=(motores->pwmRA>999)?999:motores->pwmRA;
	motores->pwmRB=(motores->pwmRB>999)?999:motores->pwmRB;

	motores->pwmLA=(motores->pwmLA<0)?0:motores->pwmLA;
	motores->pwmLB=(motores->pwmLB<0)?0:motores->pwmLB;
	motores->pwmRA=(motores->pwmRA<0)?0:motores->pwmRA;
	motores->pwmRB=(motores->pwmRB<0)?0:motores->pwmRB;

	/* Se usara el freno activo segun hoja de datos donde el otro pin
	 * debe ir constantemente en 100%, y el pwm debe ser complementario
	 */
	motores->pwmLB=999;
	motores->pwmRB=999;
	uint16_t pwmcLA=999-fabs(motores->pwmLA);
	uint16_t pwmcRA=999-fabs(motores->pwmRA);

	  __HAL_TIM_SetCompare(&Motor,TIM_CHANNEL_1,pwmcLA);	//izquierdo
	  __HAL_TIM_SetCompare(&Motor,TIM_CHANNEL_2,999);
	  __HAL_TIM_SetCompare(&Motor,TIM_CHANNEL_3,999);
	  __HAL_TIM_SetCompare(&Motor,TIM_CHANNEL_4,pwmcRA);	//derecho
	  HAL_GPIO_WritePin(EN_MOT_GPIO_Port, EN_MOT_Pin,motores->enable_PWM);
}

/**
 * @brief PID para los motores
 * @param PID: se debe mandar una estructura con los datos del pid a controlar
 * @paran setpoint: valor a cual se quiere llegar
 * @param actual: valor leido por los sensores
 * @param dt: Tiempo de muestreo segun timer puede variar
 */
float funcion_calcularPID(PID* pid,int16_t setpoint,int16_t actual,float dt)
{
	float error=(float)(setpoint-actual);
	float P=error*pid->kp;

	pid->integral=pid->integral+error;
	if(pid->integral>(pid->winup))pid->integral=(pid->winup);
	if(pid->integral<-(pid->winup))pid->integral=-(pid->winup);

	float I=pid->ki*pid->integral;

	float D=pid->kd*(error-pid->ultimoError);
	pid->ultimoError=error;

	float PIDout=P+I+D;
	if(PIDout>(pid->PWM_Maximo))PIDout=(pid->PWM_Maximo);
	if(PIDout<-(pid->PWM_Maximo))PIDout=-(pid->PWM_Maximo);
	return PIDout;
}
