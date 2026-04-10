/*
 * LIB_MPU6500.c
 *
 *  Created on: Dec 10, 2025
 *      Author: Vlady-Chuwi
 */
#include "LIB_MPU6500_SPI.h"

#include "gpio.h"
#include "spi.h"
#define		SPI_PORT			hspi2
#define 	SPI_PORT_NSS		SPI_NSS_GPIO_Port
#define		SPI_PIN_NSS			SPI_NSS_Pin
/**
 * @brief lee los valores en bruto del MPU
 * @param valoresMPU: Se debe mandar una estructura de tipo MPU6500_Init_values_t
 * @retval Se regresara valores en la misma estructura recibida
 */
void	MPU6500_Read(MPU6500_Init_Values_t* valoresMPU){
	uint8_t Reg=MPU_READ|ACCEL_XOUT_H;
	uint8_t	Val[6];
	HAL_GPIO_WritePin(SPI_PORT_NSS, SPI_PIN_NSS, 0);
	HAL_SPI_Transmit(&SPI_PORT, &Reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&SPI_PORT,&Val[0],6, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_PORT_NSS, SPI_PIN_NSS, 1);
	HAL_Delay(1);
	valoresMPU->MPU6500_ACCELX.MPU6500_uint8[1]=Val[0];
	valoresMPU->MPU6500_ACCELX.MPU6500_uint8[0]=Val[1];
	valoresMPU->MPU6500_ACCELY.MPU6500_uint8[1]=Val[2];
	valoresMPU->MPU6500_ACCELY.MPU6500_uint8[0]=Val[3];
	valoresMPU->MPU6500_ACCELZ.MPU6500_uint8[1]=Val[4];
	valoresMPU->MPU6500_ACCELZ.MPU6500_uint8[0]=Val[5];

	Reg=MPU_READ|GYRO_XOUT_H;
	HAL_GPIO_WritePin(SPI_PORT_NSS, SPI_PIN_NSS, 0);
	HAL_SPI_Transmit(&SPI_PORT, &Reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&SPI_PORT,&Val[0],6, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_PORT_NSS, SPI_PIN_NSS, 1);
	HAL_Delay(1);
	/**
	 * 	@note	Al usar unio para mover datos y convertirlos automaticamente los DATOS MSB se encuentran
	 * 			en la posicion [1] y los DATOS LSB en la posicion LSB
	 */
	valoresMPU->MPU6500_GYROX.MPU6500_uint8[1]=Val[0];
	valoresMPU->MPU6500_GYROX.MPU6500_uint8[0]=Val[1];
	valoresMPU->MPU6500_GYROY.MPU6500_uint8[1]=Val[2];
	valoresMPU->MPU6500_GYROY.MPU6500_uint8[0]=Val[3];
	valoresMPU->MPU6500_GYROZ.MPU6500_uint8[1]=Val[4];
	valoresMPU->MPU6500_GYROZ.MPU6500_uint8[0]=Val[5];

}
/**
 * @brief	Funcion para leer un solo registro de la lista
 * @param	Reg: Valor del registro de 7 bits
 */
uint8_t 	MPU6500_Read_Reg(uint8_t Reg){
	uint8_t Value=0;
	Reg=Reg|MPU_READ;
	HAL_GPIO_WritePin(SPI_PORT_NSS, SPI_PIN_NSS, 0);
	HAL_SPI_Transmit(&SPI_PORT, &Reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&SPI_PORT,&Value, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_PORT_NSS, SPI_PIN_NSS, 1);
	return Value;
}
/**
 * @brief	Funcion para escribir en un solo registro de la lista
 * @param	Reg: Valor del registro en 7 bits
 * @param	value: valor a escribir en el registro
 */
void 	MPU6500_Write_Reg(uint8_t Reg,uint8_t value){
	Reg=Reg|MPU_WRITE;
	HAL_GPIO_WritePin(SPI_PORT_NSS, SPI_PIN_NSS, 0);
	HAL_SPI_Transmit(&SPI_PORT, &Reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&SPI_PORT,&value, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_PORT_NSS, SPI_PIN_NSS, 1);
	HAL_Delay(1);
}

/**
 * @brief	Funcion para escribir en una serie de registros
 * @param	Reg: Registro donde iniciara los datos a escribir
 * @param	Value: Se debe mandar un array de datos
 * @param	len: Size del array
 */
void	MPU6500_Write(uint8_t Reg,uint8_t* value, uint8_t  len){
	Reg=Reg|MPU_WRITE;
	HAL_GPIO_WritePin(SPI_PORT_NSS, SPI_PIN_NSS, 0);
	HAL_SPI_Transmit(&SPI_PORT, &Reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&SPI_PORT, value, len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_PORT_NSS, SPI_PIN_NSS, 1);
	HAL_Delay(1);
}

/**
 * @brief	Funcion donde se inicializara los datos para el MPU y realizara los OFFSET
 * @param	offset:	Se debe ingresar una estructura para poder realizar las lecturas
 * @param	N:	Cantidad de muestras para realizar el promedio
 * @param	dps: Valor que se usara para inicializar el MPU
 * @param	g:	Valor que se usara para inicializar el MPU
 * @retval	Al terminar la funcion se mandara en la misma estructura los OFFSET calculados
 */
MPU6500_status_e	MPU6500_Init(MPU6500_Init_Values_t * offset,uint8_t N,uint8_t dps,uint8_t g){

	double PromX[N],PromY[N],PromZ[N];
	double SumaX,SumaY,SumaZ=0;

	uint8_t status_mpu=MPU6500_Read_Reg(WHO_AM_I);
	if (status_mpu!=0x70) {
		return MPU6500_fail;
	}

	MPU6500_Write_Reg(PWR_MGMT_1, 0b10000000);
	MPU6500_Write_Reg(SIGNAL_PATH_RESET, 0b00000111);
	/*	Se debe colocar a 1000dps y 16 g para realizar los offset */
	MPU6500_Write_Reg(CONFIG_ACCEL, G16);
	MPU6500_Write_Reg(CONFIG_GYRO, DPS1000);

	/*
	 * Mejorar codigo de muestreo, falla debido al overflow de datos
	 */
	for (uint8_t n = 0; n < N; ++n) {
		MPU6500_Read(offset);
		PromX[n]=(double)offset->MPU6500_ACCELX.MPU6500_int16;
		PromY[n]=(double)offset->MPU6500_ACCELY.MPU6500_int16;
		PromZ[n]=(double)offset->MPU6500_ACCELZ.MPU6500_int16;
		HAL_Delay(500);
	}
	for (uint8_t n = 0; n < N; ++n) {
		SumaX=SumaX+PromX[n];
		SumaY=SumaY+PromY[n];
		SumaZ=SumaZ+PromZ[n];
	}
	offset->MPU6500_ACCELX.MPU6500_int16=(int16_t)SumaX/N;
	offset->MPU6500_ACCELY.MPU6500_int16=(int16_t)SumaY/N;
	offset->MPU6500_ACCELZ.MPU6500_int16=(int16_t)SumaZ/N;

	/**
	 * Se lee los offset del ACCEL almacenados en el MPU para la resta correspondiente
	 */
	uint8_t hr=0;
	uint8_t lr=0;
	uint16_t valor=0;
	int16_t  resta=0;


	hr=MPU6500_Read_Reg(OFFSET_H_AX);
	lr=MPU6500_Read_Reg(OFFSET_L_AX);
	valor=(((hr<<8)|lr))>>1;
	resta=(int16_t)valor;
	/*
	 * Debido a que el registro del offset del ACCEL tiene 15 bits se debe dividir entre 2
	 * para compensar, ya que sin esto los valores no son los correctos
	 */
	resta=resta-((offset->MPU6500_ACCELX.MPU6500_int16)/2);
	hr=resta>>7;
	lr=resta<<1;
	MPU6500_Write_Reg(OFFSET_H_AX, hr);
	MPU6500_Write_Reg(OFFSET_L_AX, lr);

	hr=MPU6500_Read_Reg(OFFSET_H_AY);
	lr=MPU6500_Read_Reg(OFFSET_L_AY);
	valor=(((hr<<8)|lr))>>1;
	resta=(int16_t)valor;
	resta=resta-((offset->MPU6500_ACCELY.MPU6500_int16)/2);
	hr=resta>>7;
	lr=resta<<1;
	MPU6500_Write_Reg(OFFSET_H_AY, hr);
	MPU6500_Write_Reg(OFFSET_L_AY, lr);

	hr=MPU6500_Read_Reg(OFFSET_H_AZ);
	lr=MPU6500_Read_Reg(OFFSET_L_AZ);
	valor=(((hr<<8)|lr))>>1;
	offset->MPU6500_ACCELZ.MPU6500_int16=(offset->MPU6500_ACCELZ.MPU6500_int16)-2048;
	resta=(int16_t)valor;
	resta=resta-((offset->MPU6500_ACCELZ.MPU6500_int16)/2);
	hr=resta>>7;
	lr=resta<<1;
	MPU6500_Write_Reg(OFFSET_H_AZ, hr);
	MPU6500_Write_Reg(OFFSET_L_AZ, lr);

	/* Ahora se niega los offset del GYRO para luego mandarlos al offset del MPU*/
	offset->MPU6500_GYROX.MPU6500_int16=-(offset->MPU6500_GYROX.MPU6500_int16);
	MPU6500_Write_Reg(OFFSET_H_GX, (offset->MPU6500_GYROX.MPU6500_uint8[1]));
	MPU6500_Write_Reg(OFFSET_L_GX, (offset->MPU6500_GYROX.MPU6500_uint8[0]));

	offset->MPU6500_GYROY.MPU6500_int16=-(offset->MPU6500_GYROY.MPU6500_int16);
	MPU6500_Write_Reg(OFFSET_H_GY, (offset->MPU6500_GYROY.MPU6500_uint8[1]));
	MPU6500_Write_Reg(OFFSET_L_GY, (offset->MPU6500_GYROY.MPU6500_uint8[0]));

	offset->MPU6500_GYROZ.MPU6500_int16=-(offset->MPU6500_GYROZ.MPU6500_int16);
	MPU6500_Write_Reg(OFFSET_H_GZ, (offset->MPU6500_GYROZ.MPU6500_uint8[1]));
	MPU6500_Write_Reg(OFFSET_L_GZ, (offset->MPU6500_GYROZ.MPU6500_uint8[0]));


	/*	Y por ultimo simplemente se coloca los valores con los que funcionara el MPU*/
	MPU6500_Write_Reg(CONFIG_ACCEL, g);
	MPU6500_Write_Reg(CONFIG_GYRO, dps);
	return MPU6500_ok;

}

/**
 * @brief	Funcion para convertir los valores brutos en dato de DPS/S y G
 * @param	raw:	Estructura donde se tiene los valores en bruto
 * @param	dpsConv: Valor para la conversion segun lo ingresado en dps al iniciar
 * @param	gConv:	Valor para la conversion segun lo ingresado en g al iniciar
 * @return	convDatos:	Regresar valores ya convertidos a una estructura
 */
MPU6500_Init_float_t		MPU6500_Converter(MPU6500_Init_Values_t* raw,float dpsConv,float gConv)
{
	MPU6500_Init_float_t convDatos;
	convDatos.MPU6500_floatAX=(raw->MPU6500_ACCELX.MPU6500_int16)/gConv;
	convDatos.MPU6500_floatAY=(raw->MPU6500_ACCELY.MPU6500_int16)/gConv;
	convDatos.MPU6500_floatAZ=(raw->MPU6500_ACCELZ.MPU6500_int16)/gConv;

	convDatos.MPU6500_floatGX=(raw->MPU6500_GYROX.MPU6500_int16)/dpsConv;
	convDatos.MPU6500_floatGY=(raw->MPU6500_GYROY.MPU6500_int16)/dpsConv;
	convDatos.MPU6500_floatGZ=(raw->MPU6500_GYROZ.MPU6500_int16)/dpsConv;

	return convDatos;
}

/**
 * @brief	Funcion para obtener el Pitch en bruto
 * @param	convPitch:	Se necesita recibir una estructura donde se tenga los datos convertidos
 * @return	angulo_pitch: Retorna el pitch como tipo float
 */
float		MPU6500_Pitch(MPU6500_Init_float_t* convPitch){

	  float accelX=convPitch->MPU6500_floatAX;
	  float accelY=convPitch->MPU6500_floatAY;
	  float accelZ=convPitch->MPU6500_floatAZ;
	  float accelY_sq=accelY*accelY;
	  float accelZ_sq=accelZ*accelZ;
	  float angulo_pitch=atan2(accelX,sqrt(accelY_sq+accelZ_sq));
	  angulo_pitch=angulo_pitch*(180.0/M_PI);
	  return angulo_pitch;
}

/**
 * @brief	Funcion para obtener el Roll en bruto
 * @param	convRoll:	Se necesita recibir una estructura donde se tenga los datos convertidos
 * @return	angulo_Roll: Retorna el roll como tipo float
 */
float		MPU6500_Roll(MPU6500_Init_float_t* convRoll){
	float accelY=convRoll->MPU6500_floatAY;
	float accelZ=convRoll->MPU6500_floatAZ;
	float angulo_roll=atan2(accelY,accelZ);
	angulo_roll=angulo_roll*(180.0/M_PI);
	return angulo_roll;
}

/**
 * @brief	Funcion para obtener el Pitch y Roll usando un filtro complementario
 * @param	data:	Se necesita recibir una estructura donde se tenga los datos convertidos
 * @param	pitch:	Se debe mandar un puntero del pitch para poder regresar el valor
 * @param	roll:	Se debe mandar un puntero del roll para poder regresar el valor
 */
void MPU6500_Filtro_Complementario(MPU6500_Init_float_t* data,float *Pitch,float *Roll)
{
	/**
	 * Inicializamos variables que solo estaran si se llama a esta funcion
	 * y los cuales guardaran el estado actual
	 */
	static uint32_t ultimoTick=0;
	static float anguloRoll=0;
	static float anguloPitch=0;

	/* 1. Calcular el diferencial de tiempo (dt) */
    	uint32_t actualTick = HAL_GetTick(); // Ticks en milisegundos
		float dt = (actualTick - ultimoTick) / 1000.0f;
		ultimoTick = actualTick;

    /* Evitar errores si dt es 0 en la primera iteración */
		if (dt <= 0) return;

    /**
     *  2. Calcular ángulos solo con el Acelerómetro (Referencia estable)
     * 	Usamos las fórmulas robustas que vimos antes
     */
		float accelRoll = atan2(data->MPU6500_floatAY, data->MPU6500_floatAZ) * 180.0f / M_PI;
		float accelPitch = atan2(-data->MPU6500_floatAX,
				sqrt(data->MPU6500_floatAY * data->MPU6500_floatAY +
						data->MPU6500_floatAZ * data->MPU6500_floatAZ)) * 180.0f / M_PI;

    /**
     * 	3. Aplicar Filtro Complementario
     *	Sumamos la integración del giroscopio al ángulo anterior y corregimos con el acelerómetro
     */
		anguloRoll = ALPHA * (anguloRoll + data->MPU6500_floatGX * dt) + (1.0f - ALPHA) * accelRoll;
		anguloPitch = ALPHA * (anguloPitch + data->MPU6500_floatGY * dt) + (1.0f - ALPHA) * accelPitch;
		*Pitch=anguloPitch;
		*Roll=anguloRoll;
}

/**
 * @brief	Funcion para inicializar la estructura
 * @param	init: Estructura donde estaran los datos inicializados para el filtro
 */
void madgwickInit(MadgWick_t* init){
	init->beta=betaDef;
	init->q0=1.0f;
	init->q1=0;
	init->q2=0;
	init->q3=0;
	init->invSampleFrequency=1.0f/SampleFrequency;
	init->roll=0;
	init->pitch=0;
	init->yaw=0;
	init->anglesComputed=0;
	init->ax=0;
	init->ay=0;
	init->az=0;

	init->gx=0;
	init->gy=0;
	init->gz=0;

}

/**
 * @brief	Funcion para realizar el filtro
 * @param	dD: Estructura del filtro
 * @param	mpu6500: Estructura donde se tendra los valores ya convertidos en dps y g
 */
void madgwickUpdateIMU(MadgWick_t* qD,MPU6500_Init_float_t* mpu6500_float_DPS_G)
{

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	/* Convert gyroscope degrees/sec to radians/sec */
	qD->gx=mpu6500_float_DPS_G->MPU6500_floatGX;
	qD->gy=mpu6500_float_DPS_G->MPU6500_floatGY;
	qD->gz=mpu6500_float_DPS_G->MPU6500_floatGZ;
	qD->ax=mpu6500_float_DPS_G->MPU6500_floatAX;
	qD->ay=mpu6500_float_DPS_G->MPU6500_floatAY;
	qD->az=mpu6500_float_DPS_G->MPU6500_floatAZ;


	(qD->gx) *= DPS_TO_RADS;
	(qD->gy) *= DPS_TO_RADS;
	(qD->gz) *= DPS_TO_RADS;

/* Rate of change of quaternion from gyroscope */
		qDot1 = 0.5f * ((-qD->q1) * qD->gx - qD->q2 * qD->gy - qD->q3 * qD->gz);
		qDot2 = 0.5f * (qD->q0 * (qD->gx) + (qD->q2) * (qD->gz) - (qD->q3) * (qD->gy));
		qDot3 = 0.5f * (qD->q0 * (qD->gy) - (qD->q1) * (qD->gz) + (qD->q3) * (qD->gx));
		qDot4 = 0.5f * (qD->q0 * (qD->gz) + (qD->q1) * (qD->gy) - (qD->q2) * (qD->gx));

/* Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation) */
		if(!((qD->ax == 0.0f) && (qD->ay == 0.0f) && (qD->az == 0.0f))) {

/* Normalise accelerometer measurement */
			recipNorm = invSqrt((qD->ax) * (qD->ax) + (qD->ay) * (qD->ay) + (qD->az) * (qD->az));
			(qD->ax) *= recipNorm;
			(qD->ay) *= recipNorm;
			(qD->az) *= recipNorm;

/* Auxiliary variables to avoid repeated arithmetic */
			_2q0 = 2.0f * qD->q0;
			_2q1 = 2.0f * qD->q1;
			_2q2 = 2.0f * qD->q2;
			_2q3 = 2.0f * qD->q3;
			_4q0 = 4.0f * qD->q0;
			_4q1 = 4.0f * qD->q1;
			_4q2 = 4.0f * qD->q2;
			_8q1 = 8.0f * qD->q1;
			_8q2 = 8.0f * qD->q2;
			q0q0 = qD->q0 * qD->q0;
			q1q1 = qD->q1 * qD->q1;
			q2q2 = qD->q2 * qD->q2;
			q3q3 = qD->q3 * qD->q3;

/* Gradient decent algorithm corrective step */
			s0 = _4q0 * q2q2 + _2q2 * qD->ax + _4q0 * q1q1 - _2q1 * qD->ay;
			s1 = _4q1 * q3q3 - _2q3 * qD->ax + 4.0f * q0q0 * qD->q1 - _2q0 * qD->ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * qD->az;
			s2 = 4.0f * q0q0 * qD->q2 + _2q0 * qD->ax + _4q2 * q3q3 - _2q3 * qD->ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * qD->az;
			s3 = 4.0f * q1q1 * qD->q3 - _2q1 * qD->ax + 4.0f * q2q2 * qD->q3 - _2q2 * qD->ay;
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

/* Apply feedback step */
			qDot1 -= qD->beta * s0;
			qDot2 -= qD->beta * s1;
			qDot3 -= qD->beta * s2;
			qDot4 -= qD->beta * s3;
		}
/* Integrate rate of change of quaternion to yield quaternion */
		qD->q0 += qDot1 * qD->invSampleFrequency;
		qD->q1 += qDot2 * qD->invSampleFrequency;
		qD->q2 += qDot3 * qD->invSampleFrequency;
		qD->q3 += qDot4 * qD->invSampleFrequency;

/* Normalise quaternion */
		recipNorm = invSqrt(qD->q0 * qD->q0 + qD->q1 * qD->q1 + qD->q2 * qD->q2 + qD->q3 * qD->q3);
		qD->q0 *= recipNorm;
		qD->q1 *= recipNorm;
		qD->q2 *= recipNorm;
		qD->q3 *= recipNorm;

		qD->roll = atan2f(qD->q0*qD->q1 + qD->q2*qD->q3, 0.5f - qD->q1*qD->q1 - qD->q2*qD->q2);
		qD->pitch = asinf(-2.0f * (qD->q1*qD->q3 - qD->q0*qD->q2));
		qD->yaw = atan2f(qD->q1*qD->q2 + qD->q0*qD->q3, 0.5f - qD->q2*qD->q2 - qD->q3*qD->q3);

		qD->roll=qD->roll*57.29578f;
		qD->pitch=qD->pitch*57.29578f;
		qD->yaw=qD->yaw*57.29578f+180.0f;

}


/**
 * @brief	Funcion para obtener la raiz cuadrada inversa de un valor
 * @param	x: Valor para obtener su raiz cuadrada inversa
 * @return	y: Valor obtenido
 */
float invSqrt(float x)
{
	float halfx = 0.5f * x;
		float y = x;
		long i = *(long*)&y;
		i = 0x5f3759df - (i>>1);
		y = *(float*)&i;
		y = y * (1.5f - (halfx * y * y));
		y = y * (1.5f - (halfx * y * y));
		return y;
}









