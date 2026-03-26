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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	float valoresKLinea[3];
	float valoresKML[3];
	float valoresKMR[3];
	int32_t selectorMenu;
	int32_t selectorPID;
}Reasignarconstantes;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#undef	DEBUG
#define DEBUG		0

#define	Delay_BTN	200		// Tiempo para verificar los ADC por DMA
#define Delay_LED	50		// Tiempo para apagar LEDs
#define	ADC_VREF	3.35
#define	Volt_Proteccion_Batt 	6.0
#define	Volt_Proteccion_current 3.2
#define PWM_offset 	200
#define	Linea_setpoint	500
#define Encoder_setpoint 25
#define NumSensores 16

#define	KPLINEA 1.5
#define	KILINEA 0.001
#define	KDLINEA 0.01

#define KPML	2.0
#define KIML	0.5
#define KDML	0.0

#define KPMR	2.0
#define KIMR	0.7
#define KDMR	0.0

#define umbralMenu			100
#define umbralConstantes	10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Se inicializan todas las estructuras a usar */

volatile odometria_init_t odometria={0.0,0.0,0.0,0.0,0,0};
MPU6500_Init_Values_t 	MPU6500_Datos;
MPU6500_Init_float_t	MPU6500_Values_float;
MPU6500_status_e	MPU6500_Status;
motores_init_t	MotorSeguidor={0,0,0,0,false};
const motores_init_t	MotorStop={0,0,0,0,false};
PID	Linea={KPLINEA,KILINEA,KDLINEA,	0,0,300,PWM_offset+200};
PID	PwmBaseML={KPML,KIML,KDML,		0,0,300,PWM_offset+200};
PID	PwmBaseMR={KPMR,KIMR,KDMR,		0,0,300,PWM_offset+200};

Reasignarconstantes datosk={0};


/* Variables para los ADC */

uint32_t ADC_DMA[5];	//datos DMA
volatile uint16_t ADC_buffer[4]; //datos ya obtenidos y convertidos a 16bits
volatile uint8_t ValorBTN=0;
float ADC_Valores_Volt[4];
float valorAnteriorFiltro=0;


uint16_t PWMcorregido=0;

/* Variables para los encoders*/
const int8_t estadoTabla[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; //valor encoders de tabla de verdad
volatile uint8_t estadoAnterior_L=0;
volatile uint8_t estadoAnterior_R=0;
volatile int32_t actualTickMR,anteriorMR,deltaTicksMR,PeriodoTicksMR=0;
volatile int32_t actualTickML,anteriorML,deltaTicksML,PeriodoTicksML=0;


/* Variables para definir tiempo de espera */
uint32_t tiempoActual=0;
uint32_t tiempoAnterior=0;
uint32_t tiempoAnterior_LED=0;

/*variables para Regleta de sensores*/

static uint8_t PosicionesSensores[16]={7,6,5,4,3,2,1,0,8,9,10,11,12,13,14,15};
volatile uint8_t 	MuxSel=0;
volatile uint16_t 	RegletaSensores[16]={0};
volatile int UltimaPosicion	=500;				// var donde se almacenara la posicion en la linea
volatile unsigned long sumaPonderada = 0;
volatile unsigned long sumaLecturas = 0;
volatile long valor=0;
volatile unsigned long peso=0;

float fR=PWM_offset;
float fL=PWM_offset;
float PID_linea=0;

/*Timers para funciones*/
volatile bool		enableProg=false;
volatile bool		Timer_PID1=false;
volatile bool		Timer_PID2=false;
volatile bool		Timer_DEBUG=false;
volatile uint16_t	ContadorTimDMA_PID1=0;
volatile uint16_t	ContadorTimPID2=0;
volatile uint16_t	ContadorTimerDEBUG=0;

/* Variables para el menu */
bool 	validarPulso=false;
uint8_t valorMenu=0;
bool	validarInicio=false;

/*variables a borrar*/
int32_t a=0;
int32_t b=0;
float c=0;
float d=0;
/////////////////////
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void funcion_PID(void);
void funcion_DEBUG(void);
void funcion_InicializarMotores(void);
void funcion_ReasignarConstantesK(void);

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
  Menu_aviso(Aviso_ok);

  MPU6500_Status=MPU6500_Init(&MPU6500_Datos,10,DPS1000,G4);
  if (MPU6500_Status==MPU6500_fail) {
  	for (;;) {
  		DEBUG_Imprimir("Fallo al iniciar MPU\r\n");
  		Menu_aviso(Aviso_fallo);
  		}
  }
  DEBUG_Imprimir("Exito al iniciar MPU\r\n");

  HAL_Delay(1000);
  Menu_aviso(Apagar_LED);
 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
 MotorSeguidor=MotorStop;
 __HAL_TIM_MOE_ENABLE(&htim1);

 HAL_ADC_Start_DMA(&hadc1, &ADC_DMA[0], 4);
 HAL_ADC_Start_IT(&hadc1);
 HAL_TIM_Base_Start_IT(&htim3);

 //inhabilitamos motores
 MotorSeguidor.enable_PWM=false;
 enableProg=false;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	if(ValorBTN!=0)
	{
		if(!validarPulso)
		{
			validarPulso=true;
			valorMenu=Menu_Navegacion(ValorBTN);
			Menu_ubicacion(valorMenu);
		}
	}

	else{
		validarPulso=false;
	}
	funcion_DEBUG();


				  /* Aca se ejecutara el codigo si se dio aceptar y dependiendo el menu donde este*/
				if(Menu_Ejecucion())
				  {
					  switch (valorMenu) {
					  case Menu_Inicial:
						  break;
					  case Opcion_Calibracion_Sensores:
						  break;
					  case Opcion_Configuracion_PID_1:
						  datosk.selectorPID=0;
						  funcion_ReasignarConstantesK();
						  break;
					  case Opcion_Configuracion_PID_2:
						  datosk.selectorPID=1;
						  funcion_ReasignarConstantesK();
						  break;
					  case Opcion_Configuracion_PID_3:
						  datosk.selectorPID=2;
						  funcion_ReasignarConstantesK();
						  break;
					  case Opcion_Monitoreo:
						  break;
					  case Opcion_Iniciar_CodigoA:
						  if(!validarInicio)
						  {
							  HAL_Delay(2000);
							  funcion_InicializarMotores();
							  validarInicio=true;
						  }
						  funcion_PID();
						  break;
					  default:
								break;
										  }
				  }
				else{
					validarInicio=false;
					enableProg=false;
					MotorSeguidor=MotorStop;
					funcion_motores(&MotorSeguidor);
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance==ADC1)
	{
		ADC_buffer[0]=(uint16_t)ADC_DMA[0];
		ADC_buffer[1]=(uint16_t)ADC_DMA[1];
		ADC_buffer[2]=(uint16_t)ADC_DMA[2];
		ADC_buffer[3]=(uint16_t)ADC_DMA[3];

		/*filtramos el boton pulsado en un rango*/
		if(ADC_buffer[3]>3800 && ADC_buffer[3]<4100)
		{
			ValorBTN=BTN_DERECHA;
		}
		else if(ADC_buffer[3]>2000 && ADC_buffer[3]<2300)
				{
					ValorBTN=BTN_IZQUIERDA;
				}
		else if(ADC_buffer[3]>2400 && ADC_buffer[3]<2900)
				{
					ValorBTN=BTN_ACEPTAR;
				}
		else{
			ValorBTN=0;
		}
	}

	if(hadc->Instance==ADC2)
	{
		RegletaSensores[MuxSel] =(uint16_t) HAL_ADC_GetValue(hadc); // Lee el resultado

		if(MuxSel<16)
			{
			valor = RegletaSensores[MuxSel];
			// Se realizara una media ponderada normalizada entre 0-1000 donde 0 es iquierda y 1000 derecha
			// Umbral de ruido: 10% del valor máximo (4095 * 0.1 = 409)
			if (valor > 409) {
				// Peso del sensor (de 0 a 1000)
				peso = (MuxSel * 1000L) / (NumSensores - 1);
				sumaPonderada += peso * valor;
				sumaLecturas += valor;
				}
			MuxSel++;
			}
		else{

						UltimaPosicion = (int)(sumaPonderada / sumaLecturas);
						MuxSel=0;
						sumaLecturas=0;
						sumaPonderada=0;
						peso=0;
						valor=0;
			}
		HAL_GPIO_WritePin(S0_mux_GPIO_Port, S0_mux_Pin, (PosicionesSensores[MuxSel]&1));
		HAL_GPIO_WritePin(S1_mux_GPIO_Port, S1_mux_Pin, (PosicionesSensores[MuxSel]&2)>>1);
		HAL_GPIO_WritePin(S2_mux_GPIO_Port, S2_mux_Pin, (PosicionesSensores[MuxSel]&4)>>2);
		HAL_GPIO_WritePin(S3_mux_GPIO_Port, S3_mux_Pin, (PosicionesSensores[MuxSel]&8)>>3);
		//HAL_ADC_Start_IT(hadc);
	}

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
	   ContadorTimerDEBUG++;
	   if(ContadorTimerDEBUG>4000)
		{
		    if(!enableProg)HAL_ADC_Start_IT(&hadc1);
			Timer_DEBUG=true;
			ContadorTimerDEBUG=0;
		}
	   if(enableProg)
	   {
	   HAL_ADC_Start_IT(&hadc2);

	   ContadorTimDMA_PID1++;
	   ContadorTimPID2++;

        if(ContadorTimDMA_PID1>20)
        {
        	actualTickML=odometria.ticks_L;
			actualTickMR=odometria.ticks_R;

			deltaTicksML=actualTickML-anteriorML;
			deltaTicksMR=actualTickMR-anteriorMR;

			anteriorML=actualTickML;
			anteriorMR=actualTickMR;

			PeriodoTicksML+=deltaTicksML;
			PeriodoTicksMR+=deltaTicksMR;

        	Timer_PID1=true;
        	ContadorTimDMA_PID1=0;
        }
        if(ContadorTimPID2>400)
        {
        	Timer_PID2=true;
        	ContadorTimPID2=0;
        }

	   }

    }
}

void funcion_InicializarMotores(void)
{
	 /*esto deberia ir en una funciona para cuando se inicie dar un escalonado*/
	MotorSeguidor.enable_PWM=true;
	for(uint16_t var=0;var<PWM_offset;var+=20)
	 	{
		 MotorSeguidor.pwmLA=(uint16_t)var;
		 MotorSeguidor.pwmRA=(uint16_t)var;
		 funcion_motores(&MotorSeguidor);
		 HAL_Delay(50);
	 	}
	 odometria.ticks_L=0;
	 odometria.ticks_R=0;
	 UltimaPosicion=500;
	 actualTickMR,anteriorMR,deltaTicksMR,PeriodoTicksMR=0;
	 actualTickML,anteriorML,deltaTicksML,PeriodoTicksML=0;
	 fR=PWM_offset;
	 fL=PWM_offset;
	 PID_linea=0;

	 enableProg=true;
}

void funcion_ReasignarConstantesK(void){
	if(!validarInicio)
				  {
					  validarInicio=true;
					  odometria.ticks_L=0;
					  odometria.ticks_R=0;
					  datosk.selectorMenu=0;
				  }


				  if (odometria.ticks_L>umbralMenu) {
					  datosk.selectorMenu++;
					  odometria.ticks_L=0;
					  if(datosk.selectorMenu>2)datosk.selectorMenu=0;

					  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, !(datosk.selectorMenu&1));
					  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, !((datosk.selectorMenu&2)>>1));

				  }
				  if (odometria.ticks_L<-umbralMenu) {
					  datosk.selectorMenu--;
					  odometria.ticks_L=0;
					  if(datosk.selectorMenu<0)datosk.selectorMenu=2;
					  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, !(datosk.selectorMenu&1));
					  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, !((datosk.selectorMenu&2)>>1));

				  }

				  if (odometria.ticks_R>umbralConstantes) {
					  if(datosk.selectorPID==0){
						  datosk.valoresKLinea[datosk.selectorMenu]++;
					  }
					  else if(datosk.selectorPID==1){

						  datosk.valoresKML[datosk.selectorMenu]++;
					  }
					  else if(datosk.selectorPID==2){

						  datosk.valoresKMR[datosk.selectorMenu]++;
					  }

					  odometria.ticks_R=0;
					  HAL_GPIO_TogglePin(LED6_GPIO_Port, LED6_Pin);
				  }

				  if (odometria.ticks_R<-umbralConstantes) {
					  if(datosk.selectorPID==0){
						  datosk.valoresKLinea[datosk.selectorMenu]--;
					  }
					  else if(datosk.selectorPID==1){
						  datosk.valoresKML[datosk.selectorMenu]--;
					  }
					  else if(datosk.selectorPID==2){
						  datosk.valoresKMR[datosk.selectorMenu]--;
					  }
					  odometria.ticks_R=0;
					  HAL_GPIO_TogglePin(LED6_GPIO_Port, LED6_Pin);
				  }
				  /*
				   * Aca se  modifica el pid para cada menu
				   * y se tiene un if para evitar que sea negativo con un ++ a una variable
				   * anterior para evitar que siga restando
				   */

				  Linea.kp=KPLINEA+(datosk.valoresKLinea[0]*0.1);
				  if(Linea.kp<0)Linea.kp=0,datosk.valoresKLinea[0]++;
				  Linea.ki=KILINEA+(datosk.valoresKLinea[1]*0.001);
				  if(Linea.ki<0)Linea.ki=0,datosk.valoresKLinea[1]++;
				  Linea.kd=KDLINEA+(datosk.valoresKLinea[2]*0.001);
				  if(Linea.kd<0)Linea.kd=0,datosk.valoresKLinea[2]++;
				  PwmBaseML.kp=KPML+(datosk.valoresKML[0]*0.1);
				  if(PwmBaseML.kp<0)PwmBaseML.kp=0,datosk.valoresKML[0]++;
				  PwmBaseML.ki=KIML+(datosk.valoresKML[1]*0.001);
				  if(PwmBaseML.ki<0)PwmBaseML.ki=0,datosk.valoresKML[1]++;
				  PwmBaseML.kd=KDML+(datosk.valoresKML[2]*0.001);
				  if(PwmBaseML.kd<0)PwmBaseML.kd=0,datosk.valoresKML[2]++;
				  PwmBaseMR.kp=KPMR+(datosk.valoresKMR[0]*0.1);
				  if(PwmBaseMR.kp<0)PwmBaseMR.kp=0,datosk.valoresKMR[0]++;
				  PwmBaseMR.ki=KIMR+(datosk.valoresKMR[1]*0.001);
				  if(PwmBaseMR.ki<0)PwmBaseMR.ki=0,datosk.valoresKMR[1]++;
				  PwmBaseMR.kd=KDMR+(datosk.valoresKMR[2]*0.001);
				  if(PwmBaseMR.kd<0)PwmBaseMR.kd=0,datosk.valoresKMR[2]++;
}


void funcion_DEBUG(void)
{
	if(Timer_DEBUG)
		  			  {
		/*char buffer[30];
		sprintf(buffer,"valor menu %lu",datosk.selectorMenu);
		HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 1);

		if(datosk.selectorPID==0){

		sprintf(buffer," mr %0.3f",Linea.kp);
		HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 1);
		sprintf(buffer," mr %0.3f",Linea.ki);
		HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 1);

		sprintf(buffer," mr %0.3f ",Linea.kd);
		HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 1);
		}
		else if(datosk.selectorPID==1){

		sprintf(buffer," mr %0.3f",PwmBaseML.kp);
		HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 1);
		sprintf(buffer," mr %0.3f",PwmBaseML.ki);
		HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 1);

		sprintf(buffer," mr %0.3f ",PwmBaseML.kd);
		HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 1);
		}
		else if(datosk.selectorPID==2){

		sprintf(buffer," mr %0.3f",PwmBaseMR.kp);
		HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 1);
		sprintf(buffer," mr %0.3f",PwmBaseMR.ki);
		HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 1);

		sprintf(buffer," mr %0.3f ",PwmBaseMR.kd);
		HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 1);
		}*/

		//DEBUG_RegletaSensores(UltimaPosicion);
		//DEBUG_IMU_Conv(MPU6500_Values_float.MPU6500_floatAX,MPU6500_Values_float.MPU6500_floatAY,MPU6500_Values_float.MPU6500_floatAZ,MPU6500_Values_float.MPU6500_floatGX,MPU6500_Values_float.MPU6500_floatGY,MPU6500_Values_float.MPU6500_floatGZ);
		//DEBUG_ADC_Value(ADC_Valores_Volt[0], ADC_Valores_Volt[1], ADC_Valores_Volt[2], ADC_Valores_Volt[3]);
		//DEBUG_ADC_RAW(ADC_DMA[0], ADC_DMA[1], ADC_DMA[2], ADC_DMA[3]);

		DEBUG_Encoders(odometria.ticks_L, odometria.ticks_R, 0);

		Timer_DEBUG=false;
	}
}

void funcion_PID(void)
  {

  		  /*
  		   * Aca se realizar el primer PID cada 1ms para los motores
  		   * como tambien la lectura de DMA como su inicializacion
  		   * debe funcionar cuando se pulse y habilitar la var enableProg
  		   */

  		  if (Timer_PID1) {
  				/*obtenemos el voltaje*/
  				ADC_Valores_Volt[0]=(ADC_VREF*ADC_buffer[0])/4095;
  				ADC_Valores_Volt[1]=(ADC_VREF*ADC_buffer[1])/4095;
  				ADC_Valores_Volt[2]=(ADC_VREF*ADC_buffer[2])/4095;
  				ADC_Valores_Volt[2]=ADC_Valores_Volt[2]*3.2;
  				ADC_Valores_Volt[3]=(ADC_VREF*ADC_buffer[3])/4095;

  				/*
  				 * Antes del pid se revisa los sistemas de bloqueo por bateria baja o
  				 * motores por sobrecarga
  				 */
  				/*if(ADC_Valores_Volt[0]>Volt_Proteccion_current && PeriodoTicksMR<5){
  					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  					MotorSeguidor.enable_PWM=false;
  				}
  				if(ADC_Valores_Volt[1]>Volt_Proteccion_current && PeriodoTicksML<5){
  					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  					MotorSeguidor.enable_PWM=false;
  				}*/


  				/* PID sumamos a MR y restamos a MI*/
  				PID_linea=funcion_calcularPID(&Linea, Linea_setpoint, UltimaPosicion, 0.002f);

  				float PIDTotalMR=PWMcorregido+fR+PID_linea;
  				float PIDTotalML=PWMcorregido+fL-PID_linea;
  				c=PIDTotalML;
  				d=PIDTotalMR;
  				if(PIDTotalMR>(950))PIDTotalMR=950;
  				if(PIDTotalML>(950))PIDTotalML=950;

  				if(PIDTotalMR<20)PIDTotalMR=20;
  				if(PIDTotalMR<20)PIDTotalMR=20;

  				MotorSeguidor.pwmRA=(uint16_t)(PIDTotalMR);
  				MotorSeguidor.pwmLA=(uint16_t)(PIDTotalML);

  				funcion_motores(&MotorSeguidor);

  				HAL_ADC_Start_IT(&hadc1);	// iniciamos conversion ADC por DMA
  				Timer_PID1=false;		// Siempre limpiar bandera si no nunca entre o entrara siempre
  			}
  		  /*
  		   * Aca se realiza el PID2 a 20 ms para los encoders debido a su tiempo de retardo
  		   * y solo si la var enableProg esta habilitado
  		   */
  		  if(Timer_PID2)
  		  {
  			  	fL=funcion_calcularPID(&PwmBaseML, Encoder_setpoint, PeriodoTicksML, 0.02f);
  			  	fR=funcion_calcularPID(&PwmBaseMR, Encoder_setpoint, PeriodoTicksMR, 0.02f);
  			  	a=PeriodoTicksMR;
  			  	b=PeriodoTicksML;

  				PeriodoTicksML=0;
  			  	PeriodoTicksMR=0;

  			  	/*
  			  	 * Factor de correcion para offset PWM
  			  	 * PWMcorregido=PWMoffset*(Voltnominal/VolAactual)
  			  	 * y filtro ema
  			  	 * valorFiltrado=(alpha*valorActual)+(1-alpha)*valorAnterior
  			  	 *
  			  	 */
  			  	float alpha=0.01;
  			  	float valorFiltrado=(alpha*ADC_Valores_Volt[2])+((1-alpha)*valorAnteriorFiltro);
  			  	valorAnteriorFiltro=ADC_Valores_Volt[2];

  			  	PWMcorregido=(uint16_t)((PWM_offset*8.4)/valorFiltrado);
  			  	if(PWMcorregido>500)PWMcorregido=500;
  			  	if(PWMcorregido<10)PWMcorregido=10;


  				if(ADC_Valores_Volt[2]<Volt_Proteccion_Batt ){
  					MotorSeguidor.enable_PWM=false;
  					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  				}

  			MPU6500_Read(&MPU6500_Datos);
  			MPU6500_Values_float=MPU6500_Converter(&MPU6500_Datos, DPS1000_CONV, G4_CONV);
  			Timer_PID2=false;		// Siempre limpiar bandera si no nunca entre o entrara siempre
  		  }
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
