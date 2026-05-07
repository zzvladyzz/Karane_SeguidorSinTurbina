/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "LIB_MPU6500_SPI.h"
#include "LIB_DEBUG.h"
#include "LIB_FUNCIONES.h"
#include "LIB_MENU.h"
#include "LIB_Motores.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#undef	DEBUG
#define DEBUG		0

#define	ADC_VREF	3.36
#define	Volt_Proteccion_Batt 	6.0

#define PWM_offset 	350
#define	Linea_setpoint	500
#define tickMax 20
#define tickMin 2
#define GyroMax 400.0f


#define NumSensores 16

#define	KPLINEA 0.6
#define	KILINEA 0.00
#define	KDLINEA 0.0

#define	KPGYRO 2.3
#define	KIGYRO 0.00
#define	KDGYRO 0.00

#define KPML	1.2
#define KIML	2
#define KDML	0.0

#define KPMR	1.2
#define KIMR	2
#define KDMR	0.0

#define umbralMenu			150
#define umbralGyro			1.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Se inicializan todas las estructuras a usar */

volatile odometria_init_t odometria={0.0,0.0,0.0,0.0,0,0};
MPU6500_Init_Values_t 	MPU6500_Raw;
MPU6500_Init_float_t	MPU6500_Conv;
MPU6500_status_e	MPU6500_Status;
Motores_Init	Motor;
PID	LineaGyro={KPLINEA,KILINEA,KDLINEA,	0,0,300,GyroMax};
PID		Gyro= {KPGYRO,KIGYRO,KDGYRO,	0,0,300,400};
PID	PwmBaseML={KPML,KIML,KDML,		0,0,400,600};
PID	PwmBaseMR={KPMR,KIMR,KDMR,		0,0,400,600};



/* Variables para los ADC */

uint16_t ADC_DMA[4];	//datos DMA
volatile uint8_t ValorBTN=0;
float valorAnteriorFiltro=0;
float voltaje,A_ML,A_MR=0;


/* Variables para los encoders*/
const int8_t estadoTabla[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; //valor encoders de tabla de verdad
volatile uint8_t estadoAnterior_L=0;
volatile uint8_t estadoAnterior_R=0;
volatile int32_t actualTickMR,anteriorMR,deltaTicksMR,PeriodoTicksMR=0;
volatile int32_t actualTickML,anteriorML,deltaTicksML,PeriodoTicksML=0;



/*variables para Regleta de sensores*/

const uint8_t PosicionesSensores[16]={7,6,5,4,3,2,1,0,8,9,10,11,12,13,14,15};

volatile uint16_t 	RegletaSensores[16]={0};
int16_t UltimaPosicion	=Linea_setpoint;	// var donde se almacenara la posicion en la linea
volatile unsigned long sumaPonderada = 0;
volatile unsigned long sumaLecturas = 0;
volatile unsigned long peso=0;

/*Timers para funciones*/
volatile bool		enableProg=false;
volatile bool		Timer_DEBUG=false;
volatile bool		flagTimer10ms=false;
volatile bool 		flagStartPID=false;
volatile bool 		flagTimerGyro=false;
volatile bool 		flagStartRegleta=false;

/* Variables para el menu */
bool 	validarPulso=false;
uint8_t valorMenu=0;
bool	validarInicio=false;



float GyroObjetivo=0;
float PwmObjetivo=0;
float anguloGyro=0;
int32_t tick_L=0;
int32_t tick_R=0;
float PWMbase_MR=0;
float PWMbase_ML=0;

float setpoint_L=tickMax;
float setpoint_R=tickMax;
int16_t EscalonTick=0;
/////////////////////
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void InicializarSistema();
void InicializarValores();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  InicializarSistema();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if(flagTimerGyro)
	{

		/* Dependiendo donde se necesite se puede cambiar ahora esta a 1ms*/
		MPU6500_Read(&MPU6500_Raw);
		MPU6500_Conv=MPU6500_Converter(&MPU6500_Raw);
		if(MPU6500_Conv.MPU6500_floatGX<umbralGyro && MPU6500_Conv.MPU6500_floatGX>-umbralGyro)
		{
			MPU6500_Conv.MPU6500_floatGX=0.0f;
		}
		if(MPU6500_Conv.MPU6500_floatGY<umbralGyro && MPU6500_Conv.MPU6500_floatGY>-umbralGyro)
		{
			MPU6500_Conv.MPU6500_floatGY=0.0f;
		}
		if(MPU6500_Conv.MPU6500_floatGZ<umbralGyro && MPU6500_Conv.MPU6500_floatGZ>-umbralGyro)
		{
			MPU6500_Conv.MPU6500_floatGZ=0.0f;
		}
		anguloGyro+=(MPU6500_Conv.MPU6500_floatGZ*0.001f);
		/***********************************************************/
		flagTimerGyro=false;
		flagStartRegleta=true;
	}
	/*
	 *  Aca se ejecutara el PID una vez terminado de leer el mpu y la regleta de sensores
	 * se puede cambiar a un tiempo fijo
	 *
	 */
	if (flagStartPID ) {
		/*
		 * Primero obtenemos el PID para obtener los grados que debera girar el robot respecto
		 * al centro de la linea no superar -400g/s y +400 g/s o se puede cambia segun necesidad
		 *
		 */
		GyroObjetivo=funcion_calcularPID(&LineaGyro, 0, UltimaPosicion, 0.0015f);

		/*	Con este valor recien entrar al pid que nos importa */

		PwmObjetivo=funcion_calcularPID(&Gyro, GyroObjetivo, MPU6500_Conv.MPU6500_floatGZ, 0.0015f);


		Motor.PWM_ML=(int16_t)(PWMbase_ML-PwmObjetivo);
		Motor.PWM_MR=(int16_t)(PWMbase_MR+PwmObjetivo);
		PWM_Motores(&Motor);

		flagStartPID=false;
	}


	/*Aca calcularemos los datos que no necesitan ser procesados a alta velocidad*/
	if(flagTimer10ms)
	{
		/*obtenemos el voltaje*/
		int16_t ConvFloat=(int16_t)ADC_DMA[2];
		voltaje=(float)ConvFloat*(ADC_VREF/4095.0f);
		voltaje*=3.2641f;

		/*
		 * Para la corriente se usa Vref=(I*G*Rshunt)+Vref
		 * quedando con R=0.05 Vref=3.3v/2   G=20
		 * I=Vref-1.65v
		 * */
		ConvFloat=(int16_t)(ADC_DMA[1]-2048);
		A_ML=(float)ConvFloat*(ADC_VREF/4095.0f);

		ConvFloat=(int16_t)(ADC_DMA[0]-2048);
		A_MR=(float)ConvFloat*(ADC_VREF/4095.0f);
		A_MR*=(-1);

		//Lectura ADC para luego validacion en el menu
		if (ADC_DMA[3] > 3500)      ValorBTN = BTN_DERECHA;   // Rango 3500 - 4095
		else if (ADC_DMA[3] > 2400) ValorBTN = BTN_ACEPTAR;   // Rango 2400 - 3499
		else if (ADC_DMA[3] > 1800) ValorBTN = BTN_IZQUIERDA; // Rango 1800 - 2399
		else                     ValorBTN = 0;             // Reposo / Ruido

		static uint8_t confirmacionPulso=0;
		if(ValorBTN!=0)
		{
			confirmacionPulso++;
			if(confirmacionPulso>20)
			{
				if(!validarPulso)
				{
					validarPulso=true;
					valorMenu=Menu_Navegacion(ValorBTN);
					Menu_ubicacion(valorMenu);
				}
			}
		}
		else{
			confirmacionPulso=0;
			validarPulso=false;
		}

		/*
		 * Codigo para segun el valor de la linea obtener un factor para reducir los tick
		 * y tener una rampa que segun cual sea el valor sumar o restar
		 */

		int16_t Encoder_setpoint=0;
		int16_t valorEncoder=UltimaPosicion;
		valorEncoder=(valorEncoder<0)?valorEncoder*(-1):valorEncoder;
		if (valorEncoder > 200) {
		    // Si hay curva, el setpoint baja proporcionalmente
		    float factorEncoder = (valorEncoder - 100) / GyroMax;
		    Encoder_setpoint =(int16_t) tickMax - (factorEncoder * (tickMax - tickMin));
		} else {
		    Encoder_setpoint = tickMax;

		}
		/// revisar esto para bajar los ticks cuando este en curva o quitar lo anterior y solo
		/// usar esto ya que lo anterior no tienen mucho sentido ya que restat tick y luego
		/// volvemos a sumarlo

		if (EscalonTick < Encoder_setpoint) EscalonTick += 1;
		else if (EscalonTick > Encoder_setpoint) EscalonTick -= 1;

		/*
		 * Teniendo el valor de ticks segun la linea donde estemos se hace el pid antes
		 * leyendo los ticks del encoder y asi obteniendo el PWM base para cada motor
		 * revisarlo ya que puede meter valores negativos al enconder y no es lo optimo
		 */
        float ajusteTicks = PwmObjetivo / 40.0f;

        setpoint_L = (float)EscalonTick - ajusteTicks;
        setpoint_R = (float)EscalonTick + ajusteTicks;

		tick_R=odometria.ticks_R;
		tick_L=odometria.ticks_L;
		odometria.ticks_R=0;
		odometria.ticks_L=0;


			PWMbase_ML=funcion_calcularPID(&PwmBaseML,setpoint_L, (float) tick_L, 0.02);
			PWMbase_MR=funcion_calcularPID(&PwmBaseMR, setpoint_R, (float) tick_R, 0.02);

		flagTimer10ms=false;
	}


	if(Timer_DEBUG)
	{

		char buffer[30];
		sprintf(buffer,"angulo=%0.2f ,",anguloGyro);
		HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

		DEBUG_RegletaSensores(UltimaPosicion);
		//DEBUG_IMU_Conv(MPU6500_Conv.MPU6500_floatAX,MPU6500_Conv.MPU6500_floatAY,MPU6500_Conv.MPU6500_floatAZ,MPU6500_Conv.MPU6500_floatGX,MPU6500_Conv.MPU6500_floatGY,MPU6500_Conv.MPU6500_floatGZ);
		//DEBUG_IMU_Raw(MPU6500_Datos.MPU6500_ACCELX.MPU6500_int16,MPU6500_Datos.MPU6500_ACCELY.MPU6500_int16,MPU6500_Datos.MPU6500_ACCELZ.MPU6500_int16,	MPU6500_Datos.MPU6500_GYROX.MPU6500_int16,MPU6500_Datos.MPU6500_GYROY.MPU6500_int16,MPU6500_Datos.MPU6500_GYROZ.MPU6500_int16);

		//DEBUG_ADC_Value(voltaje, A_ML, A_MR);
		//DEBUG_ADC_RAW(ADC_DMA[0], ADC_DMA[1], ADC_DMA[2], ADC_DMA[3]);


		//DEBUG_Encoders(odometria.ticks_L, odometria.ticks_R, 0);
		//DEBUG_Encoders(tick_L,tick_R, 0);
		Timer_DEBUG=false;
	}

	/* Aca se ejecutara el codigo si se dio aceptar y dependiendo el menu donde este*/
	if(Menu_Ejecucion())
	{
		switch (valorMenu) {
		case Opcion_Calibracion_Sensores:
			break;
		case Opcion_Iniciar_CodigoA:
			if(!validarInicio)
			{
				HAL_Delay(5000);
				validarInicio=true;
				enableProg=true;
				Motor.ENABLE=true;
				InicializarValores();
			}
			break;
		}
	}
	else{
		validarInicio=false;
		enableProg=false;
		Motor.ENABLE=false;
	}



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void InicializarSistema()
{
	HAL_Delay(2000);
	  Menu_aviso(Aviso_ok);

	  MPU6500_Status=MPU6500_Init(&MPU6500_Raw,50,DPS250,G2);
	  if (MPU6500_Status==MPU6500_fail) {
	  	for (;;) {
	  		DEBUG_Imprimir("Fallo al iniciar MPU\r\n");
	  		Menu_aviso(Aviso_fallo);
	  		}
	  }
	  DEBUG_Imprimir("Exito al iniciar MPU\r\n");

	  HAL_Delay(3000);
	  Menu_aviso(Apagar_LED);
	  Inicializar_Motores(&Motor);
	 __HAL_TIM_MOE_ENABLE(&htim1);


	 HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_DMA, 4);
	 //HAL_ADC_Start(&hadc2);
	 HAL_TIM_Base_Start_IT(&htim3);

	 enableProg=false;

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		/*
		 * Se leen los enconder con tabla de verdad para sumar o restar cada uno respectivamente
		 * estadoTabla[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}
		 * se niega un lado debido a la conexion que tiene
		 */
	if(GPIO_Pin==ENCA_L_Pin||GPIO_Pin==ENCB_L_Pin)
		{
		uint8_t bitStatusL=((HAL_GPIO_ReadPin(ENCA_L_GPIO_Port, ENCA_L_Pin))?2:0) | ((HAL_GPIO_ReadPin(ENCB_L_GPIO_Port, ENCB_L_Pin))?1:0);
		odometria.ticks_L+=estadoTabla[((estadoAnterior_L<<2)|bitStatusL)];
		estadoAnterior_L=bitStatusL;
		}
	if(GPIO_Pin==ENCA_R_Pin||GPIO_Pin==ENCB_R_Pin)
		{
		uint8_t bitStatusR=((HAL_GPIO_ReadPin(ENCA_R_GPIO_Port, ENCA_R_Pin))?2:0) | ((HAL_GPIO_ReadPin(ENCB_R_GPIO_Port, ENCB_R_Pin))?1:0);
		odometria.ticks_R+=(-estadoTabla[((estadoAnterior_R<<2)|bitStatusR)]);
		estadoAnterior_R=bitStatusR;
		}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
   if (htim->Instance == TIM3) {
	   /*	Timer3 a 200us preescaler de 71 y cnt 199
	    * 				50us preescaler 71 y cnt 49 prueba
	    *	Iniciamos conversion del adc
	    *	(se podria mejorar probando tiempo mas cortos)
	    */
	  // static uint16_t	ContadorTimDMA_PID1=0;
	  /// static uint16_t	ContadorTimPID2=0;
	   static uint16_t	ContadorTimerDEBUG=0;

	   static uint16_t	ContadorTimer10ms=0;
	   static uint8_t 	MuxSel=0;
	   static uint16_t ContadorTimerGyro=0;

	   if(ContadorTimerDEBUG++>8000)
		{
		   Timer_DEBUG=true;
		   ContadorTimerDEBUG=0;
		}

	   if (ContadorTimer10ms++>200) {
		   // 1. Asegurar que el bit DMA esté activo
		   ADC1->CR2 |= ADC_CR2_DMA;
		   // 2. Configurar el disparador para SWSTART (esto es lo que le falta a tu código)
		   // EXTTRIG (bit 20) debe ser 1 y EXTSEL (bits 19:17) debe ser 111 (7 decimal)
		   ADC1->CR2 |= (0x7 << 17) | ADC_CR2_EXTTRIG;
		   // 3. Iniciar la conversión
		   ADC1->CR2 |= ADC_CR2_SWSTART;

		   flagTimer10ms=true;
		   ContadorTimer10ms=0;
	   }
	   if(ContadorTimerGyro++>20)
	   {
		   /*Timer a 1ms para el gyro*/
		   flagTimerGyro=true;
		   ContadorTimerGyro=0;
	   }


	   if(flagStartRegleta)
	   {
		   /* Terminado la lectura de Gyro a 1ms manda una bandera para empezar
		    * lectura de regleta y terminado se borra la flag para esperar otra bandera
		    * */

	   if(MuxSel<16)
	   {
		   ADC2->CR2 |= ADC_CR2_ADON;
		   ADC2->CR2 &= ~ADC_CR2_DMA;

		   // 2. Configurar el trigger a Software (SWSTART = 0x7 en EXTSEL)
		   // Esto es vital: si EXTSEL no es 7, SWSTART no hace nada.
		   ADC2->CR2 |= (0x7 << 17) | ADC_CR2_EXTTRIG;

		   // 3. Disparar
		   ADC2->CR2 |= ADC_CR2_SWSTART;

		   while (!(ADC2->SR & ADC_SR_EOC));

		   RegletaSensores[MuxSel] = (uint16_t)ADC2->DR;
		   //if(RegletaSensores[MuxSel]>RegletaMax[MuxSel]){RegletaMax[MuxSel]=RegletaSensores[MuxSel];}
		   //if(RegletaSensores[MuxSel]<RegletaMin[MuxSel]){RegletaMin[MuxSel]=RegletaSensores[MuxSel];}

		   // Se realizara una media ponderada normalizada entre 0-1000 donde 0 es iquierda y 1000 derecha
		   // Umbral de ruido: 10% del valor máximo (4095 * 0.1 = 409)
		   if (RegletaSensores[MuxSel] > 600) {
			   // Peso del sensor (de 0 a 1000)
			   peso = (MuxSel * 1000L) / (NumSensores - 1);
			   sumaPonderada += peso * RegletaSensores[MuxSel];
			   sumaLecturas += RegletaSensores[MuxSel];
		   }

		   MuxSel++;
	   }
	   else{
		   if(sumaLecturas>0)
		   {
			   UltimaPosicion = (int16_t)(sumaPonderada / sumaLecturas)-500;
		   }
		   MuxSel=0;
		   sumaLecturas=0;
		   sumaPonderada=0;
		   peso=0;
		   /* Reiniciamos tiempo regleta hasta el otro pulso y empezamos el pid */
		   flagStartRegleta=false;
		   flagStartPID=true;
	   }
	   uint8_t sel = PosicionesSensores[MuxSel];

	   S0_mux_GPIO_Port->BSRR = (sel & 0x01) ? S0_mux_Pin : (S0_mux_Pin << 16);
	   S1_mux_GPIO_Port->BSRR = (sel & 0x02) ? S1_mux_Pin : (S1_mux_Pin << 16);
	   S2_mux_GPIO_Port->BSRR = (sel & 0x04) ? S2_mux_Pin : (S2_mux_Pin << 16);
	   S3_mux_GPIO_Port->BSRR = (sel & 0x08) ? S3_mux_Pin : (S3_mux_Pin << 16);

	   for (int var = 0; var < 20; ++var) {
		   __NOP();
	   }
   }


   }
}
void InicializarValores()
{
	LineaGyro.ultimoError=0;
	LineaGyro.integral=0;
	Gyro.ultimoError=0;
	Gyro.integral=0;
	PwmBaseML.ultimoError=0;
	PwmBaseML.integral=0;
	PwmBaseMR.ultimoError=0;
	PwmBaseMR.integral=0;
	PwmObjetivo=0;
	GyroObjetivo=0;
	PWMbase_ML=0;
	PWMbase_MR=0;
	setpoint_L=0;
	setpoint_R=0;
	EscalonTick=0;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
