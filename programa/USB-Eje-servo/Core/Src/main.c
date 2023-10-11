/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "lcd_i2c.h"
#include <math.h>
#include <usbd_cdc_if.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//	Valores para el gripper sg90
#define PULSE_MIN 550
#define PULSE_MAX 2450

// Definición de las frecuencias de temporización (en Hertz)
#define TIMER_FREQUENCY 1000 // Por ejemplo, 1000 Hz (1 ms de intervalo)

// Número de motores que estás utilizando
#define NUM_MOTORS 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Motor's Struct
typedef struct {
    GPIO_TypeDef *stepPort;     // Puerto GPIO para el pin de paso (step)
    uint16_t stepPin;           // Número del pin de paso (step)
    GPIO_TypeDef *dirPort;      // Puerto GPIO para el pin de dirección (dir)
    uint16_t dirPin;            // Número del pin de dirección (dir)
    int direction;              // Dirección de movimiento (0 para positivo, 1 para negativo)
    int velocity;               // Velocidad del motor
    int microStepping;          // Configuración de microstepping
    int currentPosition;        // Posición actual del motor
    int newPosition;        	// Nueva posición del motor
    int stepCounter;            // Contador de pasos para controlar el intervalo
    int stepInterval;           // Intervalo entre pasos para controlar la velocidad
    int stopFlag;               // Bandera para detener el movimiento (0 para habilitar, 1 para detener)
} StepperMotor;

//	Cantidad de grados de libertad sin deflector final
StepperMotor motors[NUM_MOTORS];

//	Estado del deflector final
uint8_t estadoGarra = 0;

//	Vel Motors
uint16_t velMotor_X = 0;
uint16_t velMotor_Y = 0;
uint16_t velMotor_Z = 0;

//	Dir Motors
uint8_t DirM_X = 0;
uint8_t DirM_Y = 0;
uint8_t DirM_Z = 0;

// PROBAR DE CAMBIAR LOS PUNTEROS POR VALORES
//	FLAGS STOP GLOBALES
uint8_t flagStopM_X = 0;
uint8_t flagStopM_Y = 0;
uint8_t flagStopM_Z = 0;

//	FLAGS OPTICAL LIMIT SWITCH SENSOR PER AXIS GLOBALES
uint8_t opticalLimitSwitchSensor_X = 0;
uint8_t opticalLimitSwitchSensor_Y = 0;
uint8_t opticalLimitSwitchSensor_Z = 0;

///// Factor de microStepping
uint8_t microSteppingM_X = 1;
uint8_t microSteppingM_Y = 1;
uint8_t microSteppingM_Z = 1;

///// Flags de Homing
 uint8_t homeMotor_X = 0;
 uint8_t homeMotor_Y = 0;
 uint8_t homeMotor_Z = 0;
 int countHome = 0;

//	Cotador de segundos para el homming
uint8_t contSeconds = 0;

// Data to transmit CDC
char * data = "Hello World!";
uint8_t flagUsb = 0;
char buffer_rx[20];
char buffer_tx[40];
char buffer_data[5][6];
/*Buffer data
 * 0: Acción primaria
 * 1: Acción Secundaria
 * 2: Valores de velocidades
 * 3: Valores de posición
 * 4: Valor auxiliar
 */

//	LCD consts
uint8_t contador = 0;
char buf_lcd[18];

const char fig_1[8] = {0x0A, 0x0A, 0x0A, 0x00, 0x11, 0x11, 0x0E, 0x00};
const char fig_2[8] = {0x04, 0x11, 0x0E, 0x04, 0x04, 0x0A, 0x11, 0x00};
const char fig_3[8] = {0x00, 0x0A, 0x1F, 0x1F, 0x1F, 0x0E, 0x04, 0x00};
const char fig_4[8] = {0x0E, 0x1F, 0x1F, 0x0E, 0x0A, 0x11, 0x11, 0x00};
const char fig_5[8] = {0x04, 0x0E, 0x1F, 0x04, 0x04, 0x04, 0x04, 0x00};
const char fig_6[8] = {0x0E, 0x0A, 0x11, 0x11, 0x11, 0x1F, 0x1F, 0x00};
const char fig_7[8] = {0x04, 0x0E, 0x04, 0x04, 0x15, 0x15, 0x0E, 0x00};
const char fig_8[8] = {0x1F, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x1F, 0x00};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int HomingMotors(uint8_t* hmX, uint8_t* hmY, uint8_t* hmZ);
void ActivatedAll (int habilitar);
float deg2rad(float degrees);

//extern CircularBuffer<uint8_t> usbBuffer;
extern USBD_HandleTypeDef hUsbDeviceFS;
void CDC_FS_Substring(uint8_t inicioCadena, uint8_t finCadena, char* str, char* dst);

// Función para configurar el intervalo de paso en función de la velocidad del motor
void moveMotors(StepperMotor *motor, int * newPosition, int * velocity);
int targetComplete(StepperMotor *motor);

//	Función para mover el servo
void Servo_Write_angle(uint16_t theta);


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
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Apartado para inicializar el LCD
  Lcd_Init();


  ///// Configurar el bus Enable
  HAL_GPIO_WritePin(EnableMotors_GPIO_Port, EnableMotors_Pin, SET);

  // Inicialización de cada motor
  motors[0] = (StepperMotor){StepM_X_GPIO_Port, StepM_X_Pin, DirM_X_GPIO_Port, DirM_X_Pin, DirM_X, velMotor_X, microSteppingM_X, 0, 0, 0, 0, flagStopM_X};
  motors[1] = (StepperMotor){StepM_Y_GPIO_Port, StepM_Y_Pin, DirM_Y_GPIO_Port, DirM_Y_Pin, DirM_Y, velMotor_Y, microSteppingM_Y, 0, 0, 0, 0, flagStopM_Y};
  motors[2] = (StepperMotor){StepM_Z_GPIO_Port, StepM_Z_Pin, DirM_Z_GPIO_Port, DirM_Z_Pin, DirM_Z, velMotor_Z, microSteppingM_Z, 0, 0, 0, 0, flagStopM_Z};

  HAL_TIM_Base_Start_IT(&htim2);			// Iniciar el temporizador con interrupción
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim4);			// Iniciar el temporizador con interrupción
  HAL_TIM_Base_Start_IT(&htim3);


  // Envio datos al puerto USB
  CDC_Transmit_FS((uint8_t *) data, strlen (data));

  Lcd_Clear();
  Lcd_Set_Cursor(1,1);
  Lcd_Send_String("La Gaaarra!");
  Lcd_Set_Cursor(2,1);
  Lcd_Send_String("By: Mario uwu");
  Lcd_Set_Cursor(2,14);
  Lcd_Blink();
  HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(flagUsb == 1){
		  HAL_GPIO_TogglePin(LedPcb_GPIO_Port, LedPcb_Pin);
		  CDC_FS_Substring(1, 1, buffer_rx, buffer_data[0]);
//		  buffer_data[0] -> opcion
//		  if (*buffer_data[0] == 'H'){
		  if (buffer_data[0][0] == 'H'){
			  // Recordar poner ne homming el sevomotor también cerrado
			  Servo_Write_angle(0);
			  estadoGarra = 0;
			  int homeStatus = HomingMotors(&homeMotor_X, &homeMotor_Y, &homeMotor_Z);
//			  HAL_Delay(500);
			  if (homeStatus == 0){
				  sprintf(buffer_tx, "Home Status: %u\nHome exitoso!\r\n", homeStatus);
				  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
				  Lcd_Clear();
				  Lcd_Set_Cursor(1,1);
	 			  Lcd_Send_String("Home Status: OK");
	 			  Lcd_Set_Cursor(2,1);
	 			  Lcd_Send_String("X000|Y000|Z000|C");
				  HAL_Delay(150);
			  } else if (homeStatus == -1){
				  sprintf(buffer_tx, "Home Status: %u\nHome exitoso!\r\n", homeStatus);
				  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
				  Lcd_Clear();
				  Lcd_Set_Cursor(1,1);
	 			  Lcd_Send_String("Home Error: ejeX");
	 			  Lcd_Set_Cursor(2,1);
	 			  Lcd_Send_String("X???|Y000|Z000|C");
				  HAL_Delay(150);
			  } else if (homeStatus == -1){
				  sprintf(buffer_tx, "Home Status: %u\nFalla Home X!\r\n", homeStatus);
				  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
				  Lcd_Clear();
				  Lcd_Set_Cursor(1,1);
	 			  Lcd_Send_String("Home Error: ejeX");
	 			  Lcd_Set_Cursor(2,1);
	 			  Lcd_Send_String("X???|Y000|Z000|C");
				  HAL_Delay(150);
			  } else if (homeStatus == -2){
				  sprintf(buffer_tx, "Home Status: %u\nFalla Home Y!\r\n", homeStatus);
				  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
				  Lcd_Clear();
				  Lcd_Set_Cursor(1,1);
	 			  Lcd_Send_String("Home Error: ejeX");
	 			  Lcd_Set_Cursor(2,1);
	 			  Lcd_Send_String("X000|Y???|Z000|C");
				  HAL_Delay(150);
			  } else if (homeStatus == -3){
				  sprintf(buffer_tx, "Home Status: %u\nFalla Home Z!\r\n", homeStatus);
				  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
				  Lcd_Clear();
				  Lcd_Set_Cursor(1,1);
	 			  Lcd_Send_String("Home Error: ejeX");
	 			  Lcd_Set_Cursor(2,1);
	 			  Lcd_Send_String("X000|Y000|Z???|C");
				  HAL_Delay(150);
			  } else if (homeStatus == 1){
				  sprintf(buffer_tx, "Home Status: %u\nFalla en funcHome!\r\n", homeStatus);
				  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
				  Lcd_Clear();
				  Lcd_Set_Cursor(1,1);
	 			  Lcd_Send_String("ERROR Home: all!");
	 			  Lcd_Set_Cursor(2,1);
	 			  Lcd_Send_String("X???|Y???|Z???|?");
				  HAL_Delay(150);
			  }
			  HAL_Delay(1500);
		  }
		  //	CASO DE SETEO PARA VELOCIDADES GLOBALES
		  else if (buffer_data[0][0] == 'V'){
			  CDC_FS_Substring(2, 4, buffer_rx, buffer_data[2]);
			  for (int i = 0; i < NUM_MOTORS; ++i) {
				  motors[i].velocity = (uint8_t)atoi(buffer_data[2]);
			  }
			  Lcd_Set_Cursor(1,1);
//			  char v[4];
//			  sprintf(v, "%u", (uint8_t)atoi(buffer_data[2]));
// 			  Lcd_Send_String("Velocidades: %u", (uint8_t)atoi(buffer_data[2]));
			  sprintf(buffer_tx, "Velocidad globales seteadas.\r\n");
			  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
			  HAL_Delay(150);
		  }
		  //	CASO DE SETEO PARA VELOCIDADES POR EJES
		  else if (buffer_data[0][0] == 'v'){
			  for (int i = 0; i < NUM_MOTORS; ++i) {
				  motors[i].velocity = (uint8_t)atoi(buffer_data[2]);
			  }
			  CDC_FS_Substring(3, 5, buffer_rx, buffer_data[2]);
			  if (strcmp(buffer_data[1],"X")){
				  motors[0].velocity = (uint8_t)atoi(buffer_data[2]);
			  } else if (strcmp(buffer_data[1],"Y")){
				  motors[1].velocity = (uint8_t)atoi(buffer_data[2]);
			  } else if (strcmp(buffer_data[1],"Z")){
				  motors[2].velocity = (uint8_t)atoi(buffer_data[2]);
			  } else {
				  sprintf(buffer_tx, "Error en setear velocidades\r\n");
				  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
				  HAL_Delay(150);
			  }
		  }
		  //	CASO DE SETEO DE POSICIONES
		  else if (buffer_data[0][0] == 'D'){
			  char posiciones[3][5];
			  CDC_FS_Substring(2, 4, buffer_rx, posiciones[0]);
			  CDC_FS_Substring(6, 8, buffer_rx, posiciones[1]);
			  CDC_FS_Substring(9, 11, buffer_rx, posiciones[2]);
			  for (int i = 0; i < NUM_MOTORS; ++i) {
				  motors[i].newPosition = (uint8_t)atoi(buffer_data[i]);
			  }
		  }
		  //	CASO DE CONTROL DEL GRIPPER
		  else if (buffer_data[0][0] == 'P'){
			  CDC_FS_Substring(2, 4, buffer_rx, buffer_data[2]);
			  if (((uint8_t)atoi(buffer_data[2]) < 90) && ((uint8_t)atoi(buffer_data[2]) > 0)){
				  Servo_Write_angle((uint8_t)atoi(buffer_data[2]));
				  estadoGarra = 0;
				  sprintf(buffer_tx, "La Garra ha sido cerrada.\r\n");
				  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
				  Lcd_Set_Cursor(2, 16);
				  Lcd_Send_Char('C');
				  HAL_Delay(150);
			  } else if (((uint8_t)atoi(buffer_data[2]) >= 90)&& ((uint8_t)atoi(buffer_data[2]) > 0)){
				  Servo_Write_angle((uint8_t)atoi(buffer_data[2]));
				  estadoGarra = 1;
				  sprintf(buffer_tx, "La Garra ha sido abierta.\r\n");
				  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
				  Lcd_Set_Cursor(2, 16);
				  Lcd_Send_Char('A');
				  HAL_Delay(150);
			  } else {
				  sprintf(buffer_tx, "Error, valor invalido Gripper! \r\n");
				  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
				  Lcd_Set_Cursor(2, 16);
				  Lcd_Send_Char('?');
				  HAL_Delay(150);
			  }
		  }
		  //	CASO DE HABILITACION DE MOTORES
		  else if (buffer_data[0][0] == 'E'){
			  CDC_FS_Substring(2, 2, buffer_rx, buffer_data[2]);
			  if (((uint8_t)atoi(buffer_data[2]) == 1) && !((uint8_t)atoi(buffer_data[2]) < 0)){
				  ActivatedAll((uint8_t)atoi(buffer_data[2]));
				  sprintf(buffer_tx, "Se habilitaron los Motores.\r\n");
				  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
				  Lcd_Set_Cursor(1, 1);
				  Lcd_Send_String("Motores Enables!");
				  HAL_Delay(150);
			  } else if (((uint8_t)atoi(buffer_data[2]) == 0)&& !((uint8_t)atoi(buffer_data[2]) < 0)){
				  ActivatedAll((uint8_t)atoi(buffer_data[2]));
				  sprintf(buffer_tx, "Se deshabilitaron los Motores.\r\n");
				  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
				  Lcd_Set_Cursor(1, 1);
				  Lcd_Send_String("Motores Disables");
				  HAL_Delay(150);
			  } else {
				  sprintf(buffer_tx, "Error, ON/OFF motores! \r\n");
				  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
				  Lcd_Set_Cursor(1, 1);
				  Lcd_Send_String("<E ? Motores>");
				  HAL_Delay(150);
			  }
		  }
		  //	CASO DE HABILITACION DE MOTORES
		  else if (buffer_data[0][0] == 'S'){
			  ActivatedAll(-1);
			  sprintf(buffer_tx, "Se han detenido los Motores.\r\n");
			  Lcd_Set_Cursor(1, 1);
			  Lcd_Send_String("Motores Detenidos!");
			  HAL_Delay(150);
		  }
		  flagUsb = 0;
	  }
	  // Sin bandera de usb detectada
	  else {
		  if (countHome == 0){
			  Lcd_Set_Cursor(1,1);
			  Lcd_Send_String("Estado Global");
			  Lcd_Set_Cursor(2,1);
			  Lcd_Send_String("X???|Y???|Z???|?");
			  sprintf(buffer_tx, "Estado no definido\r\n");
			  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
			  HAL_Delay(750);
		  }
		  //	Caso de que el Home se haya realizado
		  else {
			  sprintf(buffer_tx, "Estado definido\r\n");
			  //	MOVER MOTORES
			  for (int k = 0; k < NUM_MOTORS; ++k) {
				  moveMotors(&motors[k], &motors[k].newPosition, &motors[k].velocity);
			  }
			  CDC_Transmit_FS((uint8_t*)buffer_tx, strlen(buffer_tx));
			  //	Actualizar LCD con datos
			  char posicionMotores[16];
			  char charGarra = '?';
			  if (estadoGarra == 0){
				  charGarra = 'C';
			  } else if (estadoGarra == 1){
				  charGarra = 'A';
			  }
			  char* position[3][4];
			  for (int i = 0; i < NUM_MOTORS; ++i) {
				  for (int j = 0; j < NUM_MOTORS; ++j) {
					  position[i][j] = '\0';
				  }
			  }
			  for (int j = 0; j < NUM_MOTORS; ++j) {
				  if (motors[j].currentPosition < 10){
					  //Agrego dos 0
					  sprintf(*position[j], "00%u", motors[j].currentPosition);
				  } else if (motors[j].currentPosition < 100){
					  //Agrego 1 cero
					  sprintf(*position[j], "0%u", motors[j].currentPosition);
				  } else {
					  //No agrego 0
					  sprintf(*position[j], "%u", motors[j].currentPosition);
				  }
			  }
			  sprintf(posicionMotores, "X%s|Y%s|Z%s|%c", position[0], position[1], position[2], charGarra);
			  Lcd_Set_Cursor(2, 1);
			  Lcd_Send_String(posicionMotores);
			  HAL_Delay(750);
		  }
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int HomingMotors(uint8_t* hmX, uint8_t* hmY, uint8_t* hmZ) {
	ActivatedAll(1);
    // Activar todos los motores y configurar velocidades
    for (int i = 0; i < NUM_MOTORS; i++) {
		motors[i].stepInterval = TIMER_FREQUENCY / (50 * motors[i].microStepping);
        motors[i].stopFlag = 1;   // Deshabilitar el movimiento
    }

    // Verificar el home de cada motor
    motors[0].stopFlag = 0;
    motors[1].stopFlag = 1;
    motors[2].stopFlag = 1;
	contSeconds = 0;
    HAL_TIM_Base_Start_IT(&htim3);
    while ((*hmX == 0 )&&(contSeconds < 3)){
        *hmX = motors[0].stopFlag;
    }
	motors[0].stopFlag = 1;
	motors[1].stopFlag = 0;
	motors[2].stopFlag = 1;
	contSeconds = 0;
    while ((*hmY == 0 )&&(contSeconds < 3)){
        *hmY = motors[1].stopFlag;
    }
    motors[0].stopFlag = 1;
    motors[1].stopFlag = 1;
    motors[2].stopFlag = 0;
	contSeconds = 0;
    while ((*hmZ == 0 )&&(contSeconds < 3)){
        *hmZ = motors[2].stopFlag;
    }
    HAL_TIM_Base_Stop_IT(&htim3);
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].velocity = 0; // Detener el motor
        motors[i].stepInterval = 0; // Detener el motor
        motors[i].currentPosition = 0; // Reiniciar la posición actual
        motors[i].stopFlag = 0;   // Habilitar el movimiento
    }
    if ((*hmX == 1 )&&(*hmY == 1 )&&(*hmZ == 1 )){// Apagar todos los motores y reiniciar posiciones
        countHome++;
        return 0;
    } else if (*hmX == 0 ){
    	return -1;
    } else if (*hmY == 0 ){
    	return -2;
    } else if (*hmZ == 0 ){
    	return -3;
    }
    return 1;
}

int targetComplete(StepperMotor *motor){
	uint8_t target = 0;
	for(int i = 0; i < NUM_MOTORS; i++){
		if (motor[i].currentPosition == motor[i].newPosition){
			target++;
		}
	}
	if (target == 3){
		return 1;
	} else {
		return 0;
	}
}

//	Función para definir comportamiento de los motores
void ActivatedAll (int habilitar){
	//		Pines Enable, Reset y Sleep comporten los mismos bus de datos
	//		para cada uno respectivamente de los distintos motores.
	//		Se recuerda que los pines son erntradas de lógica negada al A4988
	if (habilitar == -1){
	    for (int i = 0; i < NUM_MOTORS; i++) {
	        motors[i].velocity = 0; // Detener el motor
	        motors[i].stepInterval = 0; // Reiniciar la posición actual
	        motors[i].stopFlag = 1;   // Habilitar el movimiento
	    }
	}
	else if (habilitar == 1){
		HAL_GPIO_WritePin(EnableMotors_GPIO_Port, EnableMotors_Pin, RESET);			//	Se habilita el ENABLE
	}
	else if (habilitar == 0){
		HAL_GPIO_WritePin(EnableMotors_GPIO_Port, EnableMotors_Pin, SET);			//	Se Deshabilita el ENABLE
	}
}

//	Función para convertir grados a radianes
float deg2rad(float degrees) {
	return degrees * (M_PI / 180.0);
}

// Función para configurar el intervalo de paso en función de la velocidad del motor
void moveMotors(StepperMotor *motor, int *newPosition, int *velocity) {
	if (velocity != 0){
		motor->velocity = *velocity;
	}
	if (newPosition !=0){
		motor->newPosition = *newPosition;
	}
	if (motor->velocity != 0){
		if (motor->currentPosition < motor->newPosition){
			motor->direction = 0;
			motor->stopFlag = 0;
			motor->stepInterval = TIMER_FREQUENCY / (motor->velocity * motor->microStepping);
		} else if (motor->currentPosition > motor->newPosition){
			motor->direction = 1;
			motor->stopFlag = 0;
			motor->stepInterval = TIMER_FREQUENCY / (motor->velocity * motor->microStepping);
		}
	} else {
		motor->stepInterval = 0;
		motor->stopFlag = 1;
	}
}

// Función de retrollamada (callback) para la interrupción externa EXTI3
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == STOP_btn_Pin){
		HAL_GPIO_WritePin(EnableMotors_GPIO_Port, EnableMotors_Pin, SET);			//	Se Deshabilita el ENABLE
	    // Activar todos los motores y configurar velocidades
	    for (int i = 0; i < NUM_MOTORS; i++) {
	        motors[i].stopFlag = 1;   // Deshabilitar el movimiento
	    }
	}
	if (GPIO_Pin == StopM_X_Pin){
		if (flagStopM_X == 1){
			flagStopM_X = 0;
			motors[0].stopFlag = 0;
		} else {
			velMotor_X = 0;
			flagStopM_X = 1;
			motors[0].stopFlag = 1;
		}
		HAL_GPIO_TogglePin(azul_GPIO_Port, azul_Pin);
	}
	else if (GPIO_Pin == StopM_Y_Pin){
		if (flagStopM_Y == 1){
			flagStopM_Y = 0;
			motors[1].stopFlag = 0;
		} else {
			velMotor_Y = 0;
			flagStopM_Y = 1;
			motors[1].stopFlag = 1;
		}
		HAL_GPIO_TogglePin(azul_GPIO_Port, azul_Pin);
	}
	else if (GPIO_Pin == StopM_Z_Pin){
		if (flagStopM_Z == 1){
			flagStopM_Z = 0;
			motors[2].stopFlag = 0;
		} else {
			velMotor_Z = 0;
			flagStopM_Z = 1;
			motors[2].stopFlag = 1;
		}
		HAL_GPIO_TogglePin(azul_GPIO_Port, azul_Pin);
	}
  HAL_GPIO_EXTI_IRQHandler(StopM_X_Pin);  // Limpiar la bandera de interrupción EXTI3
  HAL_GPIO_EXTI_IRQHandler(StopM_Y_Pin);  // Limpiar la bandera de interrupción EXTI3
  HAL_GPIO_EXTI_IRQHandler(StopM_Z_Pin);  // Limpiar la bandera de interrupción EXTI3
  HAL_GPIO_EXTI_IRQHandler(STOP_btn_Pin);  // Limpiar la bandera de interrupción EXTI3
}

//	Función para mover el servo
void Servo_Write_angle(uint16_t theta){
	uint16_t pwm_servo;
	pwm_servo = (uint16_t)((theta-0)*(PULSE_MAX-PULSE_MIN)/(180-0)+PULSE_MIN);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pwm_servo);
}

//	FUNCIÓN PARA DIVIDIR Y COPIAR CADENAS (formatear)
void CDC_FS_Substring(uint8_t inicioCadena, uint8_t finCadena, char* str, char* dst){
	uint8_t pt = 0;
	for (uint16_t lt=inicioCadena; lt<finCadena; lt++){
		dst[pt] = str [lt];
		pt++;
	}
	dst[pt] = '\0';
	pt = 0;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
		for (int i = 0; i < NUM_MOTORS; i++) {
			StepperMotor *motor = &motors[i];
			motor->stepCounter++;
			if ((motor->newPosition != motor->currentPosition) || (countHome >= 0)){
				if ((motor->stepCounter >= motor->stepInterval)&&(motor->stepInterval != 0)) {
					motor->stepCounter = 0;
					if (motor->stopFlag == 0) {
						if (motor->direction == 0) {
							motor->currentPosition += 1;
							HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, SET);
						} else {
							motor->currentPosition -= 1;
							HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, RESET);
						}
						HAL_GPIO_WritePin(motor->stepPort, motor->stepPin, SET);
					}
					HAL_GPIO_WritePin(motor->stepPort, motor->stepPin, RESET);
				}
			}
		}

	}
	if (htim->Instance == TIM3){
		//		TIMER para contar segundos
		contSeconds++;
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

#ifdef  USE_FULL_ASSERT
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
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
