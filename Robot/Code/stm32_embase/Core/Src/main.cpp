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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
# include "msg_handler.hpp"
# include "XNucleoIHM02A1.h"
#include "BlocMoteurs.hpp"
#include <math.h>
#include <sys/wait.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define coeff_1x 1
#define coeff_1y 1
#define coeff_1z 1

#define coeff_2x 1
#define coeff_2y 1
#define coeff_2z 1

#define coeff_3x 1
#define coeff_3y 1
#define coeff_3z 1

//Caracteristiques géométriques
#define diametre_embase 0.13 //en m
#define Rayon_Roue 0.055 //en m
#define coeff_Y 0.866025404 //sqrt(3)/2

#define W_MAX 10 //en /rad/s

/*
	Utile ?
#define V_MAX 1.5 //en /m/s
#define Delay_Rotation 2000 //en ms
#define Delay_Translation 10000 //en ms
*/
#define Coeff_erreur_Y 1
#define Coeff_erreur_X 1
#define Coeff_erreur_Z 45/44

#define Vitesse_moy 0.283 //en m/s
#define Vitesse_Rotation_moy 45 //en °/s
#define Nombre_instructions 100 //Nombre d'instruction maximum avant overflow

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
/* USER CODE BEGIN PV */

//Msg handler is built over huart2 (usb) and will handle UART communication
// with the stm32_to_ros_node
MsgHandler msg_handler(&huart2);

//counter increased each Timer2 interrupt (currently 20ms)
int timer_timeout_count = 0;

//Tab of motor_speeds received from msg_handler
float input_motor_speeds[4] {0.0,0.0,0.0,0.0}; //Vx, Vy, Wz speeds

/*
volatile float Vx = 0, Vy = 0, Wz = 0; //En m/s m/s rad/s  //Utile ?
volatile float Cmde_X = 0, Cmde_Y = 0, Cmde_Z = 0; //en m, en m, en ° //Utile ?
*/
//Liste d'instructions
volatile float Liste_Instructions[Nombre_instructions][3];
//Pointeurs d'instructions
volatile uint8_t Pointeur_Instruction = 0; //Pointe vers l'instruction à executer
volatile uint8_t Pointeur_Next_Instruction = 0; //Pointe vers la prochaine instruction à executer



//Flag to control a timeout state
//Timeout occurs if no data was received from serial in a long time
bool timeout_moteurs = false;

//Class handling motors
BlocMoteurs* moteurs;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void send_print(const char* msg);
void send_float(float float_to_send);
void Set_Speeds(float V_x, float V_y, float W_z);
void Set_Step(float X, float Y, float theta_z);
void Set_Position(float X, float Y, float theta_z);
void Set_Rotation (float theta_z);
void Set_Distance(float X, float Y);
void Init_Intruction(void);
void Add_Instruction(float new_X, float new_Y, float new_Z);
void Maj_Pointeur_Instruction(BlocMoteurs* moteur_local);
void Exe_Instruction(BlocMoteurs* moteur_local);

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


  //Start Timer2 interrupt (every 20 ms here)
  HAL_TIM_Base_Start_IT(&htim2);

  //Init motor class
  moteurs = new BlocMoteurs(&hspi1, reset_shield_1_GPIO_Port, reset_shield_1_Pin, ssel1_GPIO_Port, ssel1_Pin,
		  	  	  	  	  	  	    reset_shield_2_GPIO_Port, reset_shield_2_Pin, ssel2_GPIO_Port, ssel2_Pin);
  
  //Set microstepping to 128 for smooth rotations
  moteurs->set_microstepping_mode(step_mode_t::STEP_MODE_HALF);

  moteurs->set_max_speed_moteurs(W_MAX);//Vitesse de rotation max des moteur pour eviter erreur en position
  //Set max acc to 1 rad/s^2
  //moteurs->set_max_acc_moteurs(1);

Init_Intruction();

/*
 Test de 100 instruction
for(int i = 0; i<Nombre_instructions; i+=4){
Liste_Instructions[i][0]=0;
Liste_Instructions[i][1]=0.6;
Liste_Instructions[i][2]=0;

Liste_Instructions[i+1][0]=0.6;
Liste_Instructions[i+1][1]=0;
Liste_Instructions[i+1][2]=0;

Liste_Instructions[2+i][0]=0;
Liste_Instructions[2+i][1]=-0.6;
Liste_Instructions[2+i][2]=0;

Liste_Instructions[3+i][0]=-0.6;
Liste_Instructions[3+i][1]=0;
Liste_Instructions[3+i][2]=0;
}*/

Set_Position(0,-1,0);

//////////////////////////////////////Test parcour//////////////////////////////////////////
/*
Add_Instruction(0,0.24,0);
Add_Instruction(0,0.24,0);

Add_Instruction(-0.25,0.2,0);
Add_Instruction(0.2,0.33,0);

Add_Instruction(0,0.24,0);
Add_Instruction(0,0.24,0);

Add_Instruction(-0.25,0.2,0);
Add_Instruction(0.2,0.33,0);

Add_Instruction(-0.35,0,0);
Add_Instruction(-0.5,0.4,0);
*/
/*
Liste_Instructions[0][1]=0.24;

Liste_Instructions[1][1]=0.24;

Liste_Instructions[2][1]=0.20;
Liste_Instructions[2][0]=-0.20;

Liste_Instructions[3][1]=0.33;
Liste_Instructions[3][0]=0.20;

Liste_Instructions[4][1]=0.24;

Liste_Instructions[4][1]=0.24;

Liste_Instructions[5][1]=0.24;

Liste_Instructions[6][1]=0.20;
Liste_Instructions[6][0]=-0.20;

Liste_Instructions[7][1]=0.33;
Liste_Instructions[7][0]=0.20;

Liste_Instructions[8][0]=-0.35;

Liste_Instructions[9][0]=-0.50;
Liste_Instructions[9][1]=0.40;
*/

////////////////////////////////////////////////////////////////////////////////

//Set_position_2(-0.6, 0, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 Exe_Instruction(moteurs);

    //Ask handler to fetch motor_speeds (UART_RECEIVE_IT)
	  /*
	  msg_handler.prepare_receive_motor_speeds();
	  while(1)
	  {

			if( msg_handler.get_received_motor_speeds(input_motor_speeds) != true)
			{
				if( timer_timeout_count >= 5) // 5*TIM2_INTERRUPT_PERIOD(20m) -> Timeout 100ms
				{
					moteurs->motors_stop_soft_hiz(); // stop motors
					timeout_moteurs = true;
					break;
				}
			}
			else
			{
				moteurs->motors_on(); // toggle motors on in case it was stopped with motor_stop
				timeout_moteurs = false;
				break;
			}

	  }
	  if(timeout_moteurs == false)
	  {
      //Apply received_motor_speeds
		  moteurs->commande_vitesses_absolues(input_motor_speeds[front_left], input_motor_speeds[front_right], input_motor_speeds[back_left], input_motor_speeds[back_right]);
		  //Measure and send motor speeds
		  msg_handler.send_motor_speeds(moteurs->mesure_vitesses_rad());
	  }
    //reset counter for next loop
	  timer_timeout_count = 0;
*/
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

//Permet d'initialiser à 0 les instructions de la liste d'instruction (Liste_Instructions)
void Init_Intruction(void){
	for(int i =0; i<10; i++){
		for(int j =0; j<3; j++){
			Liste_Instructions[i][j]=0;
		}
	}
}
//Permet d'ajputer une instruction à la liste d'instruction (Liste_Instructions)
void Add_Instruction(float new_X, float new_Y, float new_Z){
	Pointeur_Next_Instruction+=1;
	Pointeur_Next_Instruction=Pointeur_Next_Instruction%Nombre_instructions;

	//Mise à jour des instructions
	Liste_Instructions[Pointeur_Next_Instruction][0]= new_X;
	Liste_Instructions[Pointeur_Next_Instruction][1]= new_Y;
	Liste_Instructions[Pointeur_Next_Instruction][2]= new_Z;
}
//Permet de mettre à jour le pointeur d'instruction (Pointeur_Instruction) et de réinitialiser l'instruction passée
void Maj_Pointeur_Instruction(BlocMoteurs* moteur_local){

	Liste_Instructions[Pointeur_Instruction][0] = 0;
	Liste_Instructions[Pointeur_Instruction][1] = 0;
	Liste_Instructions[Pointeur_Instruction][2] = 0;

	Pointeur_Instruction+=1;
	Pointeur_Instruction=Pointeur_Instruction%Nombre_instructions;

}
//Permet d'éxécuter toute les instruction de la file d'instruction de deplacement (Liste_Instructions)
void Exe_Instruction(BlocMoteurs* moteur_local){
	volatile float exe_X, exe_Y, exe_Z;

	exe_X = Liste_Instructions[Pointeur_Instruction][0];
	exe_Y = Liste_Instructions[Pointeur_Instruction][1];
	exe_Z = Liste_Instructions[Pointeur_Instruction][2];

	Set_Position(exe_X, exe_Y, exe_Z);
	Maj_Pointeur_Instruction(moteur_local);

}
/*Permet d'ordonnée aux moteurs  de tourner à une certaine vitesse pour faire avancer le robots au vitesse Vx,Vy et Wz*/
void Set_Speeds(float V_x, float V_y, float W_z){
	float W_a=1/Rayon_Roue*(coeff_1x*V_x*0.5-coeff_1y*V_y*coeff_Y-diametre_embase*W_z*coeff_1z);
	float W_c=1/Rayon_Roue*(coeff_3x*V_x*0.5+V_y*coeff_Y*coeff_3y-diametre_embase*W_z*coeff_3z);
	float W_b=-1/Rayon_Roue*(coeff_2x*1*V_x-diametre_embase*W_z*coeff_2z);
	//float W_b=-1/Rayon_Roue*(-coeff_2x*1*V_x-diametre_embase*W_z*coeff_2z);//Previous
	  moteurs->motors_on();
	  moteurs->commande_vitesses_absolues(W_a, W_b, W_c, 0.0);
}
/*Permet d'ordonnée aux moteurs d'effectuer un certain nombre de step pour faire se delapcer le robot au point (X,Y) en (m,m)
et de s'orienté d'un angle en ° autour de son axe z*/
void Set_Step(float X, float Y, float theta_z){
	//Conversion en m
	X*=4 ; //Conversion pour que Set-position(1,0,0) donne une translation de 1m suivant X
	Y*=4 ; //Conversion pour que Set-position(0,1,0) donne une translation de 1m suivant Y
	theta_z*=0.070777; //Conversion pour que Set_Position(0,0,360) donne une rotation de 360° autour de Z

	//Angles en radians
	float theta_a= 1/Rayon_Roue*(coeff_1x*X*0.5-coeff_1y*Y*coeff_Y-diametre_embase*theta_z*coeff_1z);
	float theta_c= 1/Rayon_Roue*(coeff_3x*X*0.5+Y*coeff_Y*coeff_3y-diametre_embase*theta_z*coeff_3z);
	float theta_b= 1/Rayon_Roue*(coeff_2x*1*X-diametre_embase*theta_z*coeff_2z);

	float step_a = moteurs->rad_to_step(theta_a);
	float step_b = moteurs->rad_to_step(theta_b);
	float step_c = moteurs->rad_to_step(theta_c);

	StepperMotor::direction_t dir_a = FWD;
	StepperMotor::direction_t dir_b = FWD;
	StepperMotor::direction_t dir_c = FWD;

	//Gestion des directions par défaut: avancer sinon, si nb de step negatif, inversion du nb de steps et direction arriere
	if(step_a<0){
			step_a = 0-step_a;
			dir_a = BWD;
	}
	if(step_b<0){
				step_b = 0-step_b;
				dir_b = BWD;
		}
	if(step_c<0){
				step_c = 0-step_c;
				dir_c = BWD;
		}

	moteurs->commande_step_indiv(step_a, dir_a, step_b, dir_b, step_c, dir_c, 0, FWD);
}
//Permet d'ordonnée au robot de se deplacer au point (X,Y) en (m,m)
void Set_Distance(float X, float Y){
	volatile float D =0;
	volatile float theta =0;
	uint32_t delay_translation = 0;
	uint32_t delay_rotation = 0;

	if((Y==0)&&(X!=0)){
		if(X<0){
			theta = -90;
			D=-X;

		}
		else{
			theta = 90;
			D=X;

		}

	}
	else if((Y!=0)&&(X==0)){
		if(Y<0){
					theta = 180;
					D=-Y;

				}
		else	{
					theta = 0;
					D=Y;

				}
	}
	else{
		D = sqrt(pow(X*Coeff_erreur_X,2)+pow(Y*Coeff_erreur_Y,2));//Distance à parcourir en m
		theta = atan2(X,Y)*180.0 / M_PI;//Angle vers lequel s'orienté en °

	}
	if(abs(D)<0.41){
		delay_translation = 1500;
		delay_rotation = 2500;
	}

	else if(abs(D)<0.75){
		delay_translation = 3000;
		delay_rotation = 2500;
	}
	else{
		delay_translation = abs(D/Vitesse_moy * 1000);
		delay_rotation = abs(theta/Vitesse_Rotation_moy * 1000);
	}
	if((X==0)&&(Y==0)){
		delay_translation = 0;
		delay_rotation = 0;
	}
	if(X==0){
		delay_rotation = 0;
	}

	Set_Rotation(theta);
	HAL_Delay(delay_rotation);
	Set_Step(0, D,0);
	HAL_Delay(delay_translation);
	Set_Rotation(-theta);
	HAL_Delay(delay_rotation);

}
//Permet de faire une rotation d'un angle en ° autour de l'axe z du robot
void Set_Rotation (float theta_z){
	Set_Step(0,0,theta_z * Coeff_erreur_Z);

}
/*Permet de d'ordonnée au robots de se deplacer en un point (X,Y) en (m,m)
puis de s'orienter d'un angle en ° autour de son axe z*/
void Set_Position(float X, float Y, float theta_z){
	moteurs->motors_on();
	Set_Distance(X,Y);
	Set_Rotation(theta_z);
	moteurs->motors_stop_hard();

}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1800000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ssel1_Pin|LD2_Pin|ssel2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, reset_shield_2_Pin|reset_shield_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ssel1_Pin LD2_Pin ssel2_Pin */
  GPIO_InitStruct.Pin = ssel1_Pin|LD2_Pin|ssel2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : reset_shield_2_Pin reset_shield_1_Pin */
  GPIO_InitStruct.Pin = reset_shield_2_Pin|reset_shield_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//Redirect Uart callback to msg_handler if huart is huart2 peripheral
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		msg_handler.process_txclpt_callback();
	}


}

//Redirect Uart callback to msg_handler if huart is huart2 peripheral
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if( huart == &huart2)
	{
		msg_handler.process_rxclpt_callback();
	}

}

//Timer 2 interrupt (every 20ms)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim2 )
  {

	  timer_timeout_count++;
	  if (timer_timeout_count > 10) //in case of a timeout, reset flags that may lock the logic of the msg_handler 
	  {
		  msg_handler.unlock_timeout();
	  }

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
