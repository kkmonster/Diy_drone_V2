/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#define beta                        0.1f     
#define ACCELEROMETER_SENSITIVITY   8192.0f  
#define GYROSCOPE_SENSITIVITY       65.5f  
#define Compass_SENSITIVITY       	1090.0f
#define M_PI                        3.14159265359f	    
#define M_PIf       								3.14159265358979323846f
#define sampleFreq                  500.0f     			    // 200 hz sample rate!   
#define limmit_I                    300.0f     

#define roll_offset    0.0f     
#define pitch_offset   0.0f

#define gx_diff 		-193
#define gy_diff 		80
#define gz_diff 		-70

//#define gx_diff 		0
//#define gy_diff 		0
//#define gz_diff 		0

#define Kp_yaw      3.5f
#define Ki_yaw      0.001f
#define Kd_yaw      0.0f

#define Kp_pitch		16.0f
#define Ki_pitch    0.001f
#define Kd_pitch    9.0f

#define Kp_roll	    Kp_pitch
#define Ki_roll  		Ki_pitch
#define Kd_roll  		Kd_pitch



#define ENABLE_MPU6000  HAL_GPIO_WritePin(CS_MPU6000_GPIO_Port, CS_MPU6000_Pin, GPIO_PIN_RESET)
#define DISABLE_MPU6000 HAL_GPIO_WritePin(CS_MPU6000_GPIO_Port, CS_MPU6000_Pin, GPIO_PIN_SET)

#define ENABLE_HMC5983  HAL_GPIO_WritePin(CS_MHC5983_GPIO_Port, CS_MHC5983_Pin, GPIO_PIN_RESET)
#define DISABLE_HMC5983 HAL_GPIO_WritePin(CS_MHC5983_GPIO_Port, CS_MHC5983_Pin, GPIO_PIN_SET)

#define MPU6000_SPI &hspi1
#define HMC5983_SPI &hspi1

#define i2c_buffer_size 48




#define ARM_MATH_CM0
//#include "arm_math.h" 
#include "math.h" 
#include "MPU6000.h"
#include "HMC5983.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int16_t rawMagx_X = 1;
int16_t rawMagx_Y = 1;
int16_t rawMagx_Z = 1;
int16_t rawAccx_X = 1;
int16_t rawAccx_Y = 1;
int16_t rawAccx_Z = 1;
int16_t rawGyrox_X = 1;
int16_t rawGyrox_Y = 1;
int16_t rawGyrox_Z = 1;

int16_t magneticDeclination = 0;
float Ref_yaw=0, Ref_pitch=0, Ref_roll=0 ;
int16_t q_yaw, q_pitch, q_roll;                               		// States value
float q0=1, q1=0, q2=0, q3=0;
float T_center =0, yaw_center=0;
float Error_yaw=0, Errer_pitch=0, Error_roll=0; 								//States Error
float Sum_Error_yaw=0, Sum_Error_pitch=0, Sum_Error_roll=0;     // Sum of error
float D_Error_yaw=0, D_Error_pitch=0, D_Error_roll=0; 					// error dot
float Del_yaw=0, Del_pitch=0, Del_roll=0;												// Delta states value for rotate axis
float t_compensate = 0;
float T_center_minus = 0;
float y_roll=0, y_pitch=0, y0_roll=0, y0_pitch=0 ; 

float rMat[3][3] = {0};


uint8_t  I2C_rx_buffer = 0;
uint8_t  I2C_rx_data[i2c_buffer_size] = {0};
uint8_t  I2C_rx_data_index = 0;

uint16_t watchdog  = 0;

/* USER CODE for SPPM Receiver  */

uint8_t	index= 0 ;
int16_t	ch1=0,ch2=0,ch3=0,ch4=0;                 
int16_t	motor_A=0, motor_B=0, motor_C=0, motor_D=0 ;// Motors output value 
uint8_t rx_tmp[14] = {0};
float a, b, c;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void initMPU6000(void);
void initHMC5983(void);
void Read_MPU6000(void);
void Read_HMC5983(void);
void PID_controller(void);
void Drive_motor_output(void);
void Interrupt_call(void);
void AHRS(void);
float Smooth_filter(float alfa, float new_data, float prev_data);
float invSqrt(float x) ;
void imuComputeRotationMatrix(void);
float sq(float x);
float constrain(float x, float lower_b, float upper_b);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
	HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
	
	DISABLE_HMC5983;
	DISABLE_MPU6000;
	
	initMPU6000();
	initHMC5983();
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	
	HAL_Delay(1000);
	
	HAL_I2C_Slave_Receive_DMA(&hi2c1, &I2C_rx_buffer, 1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//		Read_HMC5983();
//		HAL_Delay(10);
//		
//		
//		Interrupt_call();
//		HAL_Delay(5);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLED;
  HAL_SPI_Init(&hspi1);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2399;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_MspPostInit(&htim3);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_MHC5983_Pin|CS_MPU6000_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_MHC5983_Pin CS_MPU6000_Pin */
  GPIO_InitStruct.Pin = CS_MHC5983_Pin|CS_MPU6000_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_MPU6000_Pin */
  GPIO_InitStruct.Pin = INT_MPU6000_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(INT_MPU6000_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void initMPU6000(void)
{
		uint8_t tmp = 0;
    DISABLE_MPU6000;

    HAL_Delay(150);
	
    ENABLE_MPU6000;
		tmp = MPU6000_PWR_MGMT_1;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);          // Device Reset
    tmp = BIT_H_RESET;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);
    DISABLE_MPU6000;
	
    HAL_Delay(150);

    ENABLE_MPU6000;
    tmp = MPU6000_PWR_MGMT_1;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);          // Clock Source PPL with Z axis gyro reference
    tmp = MPU_CLK_SEL_PLLGYROZ;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);
    DISABLE_MPU6000;

    HAL_Delay(1);

    ENABLE_MPU6000;
    tmp = MPU6000_USER_CTRL;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);           // Disable Primary I2C Interface
    tmp = BIT_I2C_IF_DIS;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);
    DISABLE_MPU6000;

    HAL_Delay(1);

    ENABLE_MPU6000;
    tmp = MPU6000_ACCEL_CONFIG;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);        // Accel +/- 4 G Full Scale
    tmp = BITS_FS_4G;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);
    DISABLE_MPU6000;

    HAL_Delay(1);

    ENABLE_MPU6000;
    tmp = MPU6000_GYRO_CONFIG;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);         // Gyro +/- 500 DPS Full Scale
    tmp = BITS_FS_500DPS;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);
    DISABLE_MPU6000;

    HAL_Delay(1);

    ENABLE_MPU6000;
    tmp = MPU6000_CONFIG;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);       // Accel and Gyro DLPF Setting
    tmp = BITS_DLPF_CFG_42HZ;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);
    DISABLE_MPU6000;

    HAL_Delay(1);
		
    ENABLE_MPU6000;
    tmp = MPU6000_SMPLRT_DIV;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);          // Accel & Gyro Sample Rate 200 Hz
    tmp = 0x01;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);
    DISABLE_MPU6000;

    HAL_Delay(1);

    ENABLE_MPU6000;
    tmp = MPU6000_INT_ENABLE;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);          // DATA_RDY_EN
    tmp = 0x01;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);
    DISABLE_MPU6000;

    HAL_Delay(100);
}

void initHMC5983(void)
{
		uint8_t tmp = 0;
    DISABLE_HMC5983;

    HAL_Delay(150);
	
    ENABLE_HMC5983;
		tmp = 0x00;
    HAL_SPI_Transmit(HMC5983_SPI, &tmp, 1, 10);          // Write Configuration Register A 
    tmp = 0x70 ;
    HAL_SPI_Transmit(HMC5983_SPI, &tmp, 1, 10);
    DISABLE_HMC5983;

		HAL_Delay(1);

    ENABLE_HMC5983;
		tmp = 0x01;
    HAL_SPI_Transmit(HMC5983_SPI, &tmp, 1, 10);          // Write Configuration Register B 
    tmp = 0x20;
    HAL_SPI_Transmit(HMC5983_SPI, &tmp, 1, 10);
    DISABLE_HMC5983;

		HAL_Delay(1);
		
		ENABLE_HMC5983;
		tmp = 0x02;
    HAL_SPI_Transmit(HMC5983_SPI, &tmp, 1, 10);          // Write Mode Register
    tmp = 0x00;
    HAL_SPI_Transmit(HMC5983_SPI, &tmp, 1, 10);
    DISABLE_HMC5983;
	
    HAL_Delay(100);
}






void Read_MPU6000(void)
{
//		uint8_t rx_tmp[14] = {0};
	  ENABLE_MPU6000;
    uint8_t tmp = 0x3b | 0x80;
    HAL_SPI_Transmit(MPU6000_SPI, &tmp, 1, 10);          // DATA_RDY_EN
    HAL_SPI_Receive(MPU6000_SPI, rx_tmp, 14, 10);
    DISABLE_MPU6000;
				
		rawAccx_X  =  ((int16_t)rx_tmp[0])<<8 | (int16_t)rx_tmp[1];
		rawAccx_Y  =  ((int16_t)rx_tmp[2])<<8 | (int16_t)rx_tmp[3];
		rawAccx_Z  =  ((int16_t)rx_tmp[4])<<8 | (int16_t)rx_tmp[5];
		rawGyrox_X = ((int16_t)rx_tmp[8])<<8 | (int16_t)rx_tmp[9];
		rawGyrox_Y = ((int16_t)rx_tmp[10])<<8 | (int16_t)rx_tmp[11];
		rawGyrox_Z = ((int16_t)rx_tmp[12])<<8 | (int16_t)rx_tmp[13];
	  
//		a = Smooth_filter(0.1f, rawGyrox_X, a);
//		b = Smooth_filter(0.1f, rawGyrox_Y, b);	
//		c = Smooth_filter(0.1f, rawGyrox_Z, c);
	
		rawGyrox_X -= gx_diff;
		rawGyrox_Y -= gy_diff;
		rawGyrox_Z -= gz_diff;
}

void Read_HMC5983(void)
{

	ENABLE_HMC5983;
	uint8_t tmp = 0x03 | 0xC0; 
	HAL_SPI_Transmit(HMC5983_SPI, &tmp, 1, 10);          // Write Mode Register
	HAL_SPI_Receive(HMC5983_SPI, rx_tmp, 6, 10);
	DISABLE_HMC5983;
	
	int16_t _rawMagx_X =  ((int16_t)rx_tmp[0])<<8 | (int16_t)rx_tmp[1];
	int16_t _rawMagx_Y =  ((int16_t)rx_tmp[2])<<8 | (int16_t)rx_tmp[3];
	int16_t _rawMagx_Z =  ((int16_t)rx_tmp[4])<<8 | (int16_t)rx_tmp[5];
	

	if (rawMagx_X == -4096 || rawMagx_Y == -4096 || rawMagx_Z == -4096)
	{
		rawMagx_X = 1;
		rawMagx_Y = 1;
		rawMagx_Z = 1;
	}
	else
	{
		rawMagx_X = _rawMagx_X;
		rawMagx_Y = _rawMagx_Y;
		rawMagx_Z = _rawMagx_Z;		
	}
}
void PID_controller(void)
{
	static float T_center_buffer;
	float Buf_D_Error_yaw   =Error_yaw;
	float Buf_D_Errer_pitch =Errer_pitch;
	float Buf_D_Error_roll  =Error_roll; 
     
	T_center_buffer    = (float)ch3 *   18.0f;
	
	T_center = Smooth_filter(0.4f, T_center_buffer, T_center);
	T_center = 400;
	//Error_yaw 	= (float)ch4 * 3.0f   -  (float)q_yaw/10.0f;
	
	Error_yaw 	= -(float)ch4 * 3.0f   + ((float)rawGyrox_Z)/GYROSCOPE_SENSITIVITY;
	Errer_pitch = (float)ch2 * -0.30f - ((float)q_pitch/10.0f - pitch_offset)	;
	Error_roll 	= (float)ch1 * 0.30f  - ((float)q_roll/10.0f - roll_offset)	;
	
  // protect  wind-up
	Sum_Error_yaw =   constrain((Sum_Error_yaw   + (Error_yaw   /sampleFreq)), -250.0f / Ki_yaw,   250.0f / Ki_yaw) ;
	Sum_Error_pitch = constrain((Sum_Error_pitch + (Errer_pitch /sampleFreq)), -250.0f / Ki_pitch, 250.0f / Ki_pitch) ;
	Sum_Error_roll =  constrain((Sum_Error_roll  + (Error_roll  /sampleFreq)), -250.0f / Ki_roll,  250.0f / Ki_roll) ;
	
	
	D_Error_yaw =  (Error_yaw-Buf_D_Error_yaw)    *sampleFreq ;
	D_Error_pitch =(Errer_pitch-Buf_D_Errer_pitch)*sampleFreq;
	D_Error_roll = (Error_roll-Buf_D_Error_roll)*sampleFreq;

	Del_yaw		= (Kp_yaw   * Error_yaw)		+ (Ki_yaw	  * Sum_Error_yaw)   + constrain((Kd_yaw * D_Error_yaw), -300, 300);
	Del_pitch	= (Kp_pitch * Errer_pitch)	+ (Ki_pitch	* Sum_Error_pitch) + constrain((Kd_pitch * D_Error_pitch), -300, 300);
	Del_roll	= (Kp_roll  * Error_roll)		+ (Ki_roll	* Sum_Error_roll)  + constrain((Kd_roll * D_Error_roll), -300, 300);

	motor_A = T_center + constrain((+Del_pitch	+Del_roll +Del_yaw), -1250, 1250);
	motor_B = T_center + constrain((+Del_pitch	-Del_roll -Del_yaw), -1250, 1250) ;
	motor_C = T_center + constrain((-Del_pitch	-Del_roll +Del_yaw), -1250, 1250) ;
	motor_D = T_center + constrain((-Del_pitch	+Del_roll -Del_yaw), -1250, 1250) ;

}

void Drive_motor_output(void)
{
	
//  motor_A = 100 ;
//	motor_B = 100 ;
//	motor_C = 100 ;
//	motor_D = 100 ;
	
	// limmit output max, min
	if(motor_A < 0) motor_A = 0 ;
	if(motor_B < 0) motor_B = 0 ;
	if(motor_C < 0) motor_C = 0 ;
	if(motor_D < 0) motor_D = 0 ;
	
	if(motor_A > 2399) motor_A = 2399 ;
	if(motor_B > 2399) motor_B = 2399 ;
	if(motor_C > 2399) motor_C = 2399 ;
	if(motor_D > 2399) motor_D = 2399 ;
	
	TIM2 ->CCR1 = motor_D ;
	TIM2 ->CCR2 = motor_A ;
	TIM3 ->CCR1 = motor_B ;
	TIM3 ->CCR2 = motor_C ;
}

void Interrupt_call(void)
{
			/* Read data from sensor */
		Read_MPU6000();
		Read_HMC5983();

		AHRS(); 
		/* Controller */
		PID_controller();
	  watchdog = 10;
    if (watchdog > 0) watchdog --;
	
		if((T_center < 50) || (watchdog == 0))		
		{
            
//			Sum_Error_yaw=0;
//			Sum_Error_pitch=0;
//			Sum_Error_roll=0;        
			
			motor_A=0;
			motor_B=0;
			motor_C=0;
			motor_D=0;
		}
        
	Drive_motor_output(); 
}

void AHRS()
{
	  float dt = 0.002f; 
		float gx = (((float)rawGyrox_X)/GYROSCOPE_SENSITIVITY)*(M_PIf/180.0f);
		float gy = (((float)rawGyrox_Y)/GYROSCOPE_SENSITIVITY)*(M_PIf/180.0f);
		float gz = (((float)rawGyrox_Z)/GYROSCOPE_SENSITIVITY)*(M_PIf/180.0f);
		float ax = ((float)rawAccx_X)/ACCELEROMETER_SENSITIVITY;
		float ay = ((float)rawAccx_Y)/ACCELEROMETER_SENSITIVITY;
		float az = ((float)rawAccx_Z)/ACCELEROMETER_SENSITIVITY;
    float my =-((float)rawMagx_X)/Compass_SENSITIVITY;
		float mx =-((float)rawMagx_Y)/Compass_SENSITIVITY;
		float mz =-((float)rawMagx_Z)/Compass_SENSITIVITY;
	
//		gx = 0;
//		gy = 0;
//		gz = 0;
//		ax = 0;
//		ay = 0;
//		az = 0;
//    mx = ((float)rawMagx_X)/Compass_SENSITIVITY;
//		my = ((float)rawMagx_Y)/Compass_SENSITIVITY;
//		mz = ((float)rawMagx_Z)/Compass_SENSITIVITY;

		static uint8_t useMag = 0;
		static uint8_t useAcc = 1;
    float recipNorm;
    float hx, hy, bx;
    float ex = 0, ey = 0, ez = 0;
    float qa, qb, qc;

    // Use measured magnetic field vector
    recipNorm = sq(mx) + sq(my) + sq(mz);
    if (useMag && (recipNorm > 0.01f)) {
        // Normalise magnetometer measurement
        recipNorm = invSqrt(recipNorm);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
        // This way magnetic field will only affect heading and wont mess roll/pitch angles

        // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
        // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
        hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
        hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
			
				a = hx;
				b = hy;
				c = atan2f(a,b)* (1800.0f / M_PIf);
        bx = sqrtf(hx * hx + hy * hy);

        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
        float ez_ef = -(hy * bx);

        // Rotate mag error vector back to BF and accumulate
        ex += rMat[2][0] * ez_ef;
        ey += rMat[2][1] * ez_ef;
        ez += rMat[2][2] * ez_ef;
    }

    // Use measured acceleration vector
    recipNorm = sq(ax) + sq(ay) + sq(az);
    if (useAcc && (recipNorm > 0.01f)) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(recipNorm);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += (ay * rMat[2][2] - az * rMat[2][1]);
        ey += (az * rMat[2][0] - ax * rMat[2][2]);
        ez += (ax * rMat[2][1] - ay * rMat[2][0]);
    }

    // Calculate kP gain. If we are acquiring initial attitude (not armed and within 20 sec from powerup) scale the kP to converge faster
    float dcmKpGain = 0.25f;

    // Apply proportional and integral feedback
    gx += dcmKpGain * ex;
    gy += dcmKpGain * ey;
    gz += dcmKpGain * ez;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(sq(q0) + sq(q1) + sq(q2) + sq(q3));
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();
		
		/* Compute pitch/roll angles */

    q_pitch =  (atan2f(rMat[2][1], rMat[2][2]) * (1800.0f / M_PIf));
    q_roll  = -(((0.5f * M_PIf) - acosf(-rMat[2][0])) * (1800.0f / M_PIf));
    q_yaw   =  ((atan2f(rMat[1][0], rMat[0][0]) * (1800.0f / M_PIf) + magneticDeclination));
		
		if (q_yaw < 0) q_yaw += 3600;
}
float constrain(float x, float lower_b, float upper_b)
{
	if(x < lower_b) x = lower_b;
	if(x > upper_b) x = upper_b;
	return x;
}


float sq (float x)
 {
	 return x*x;
 }
void imuComputeRotationMatrix(void)
{
    float q1q1 = sq(q1);
    float q2q2 = sq(q2);
    float q3q3 = sq(q3);
    
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}
float Smooth_filter(float alfa, float new_data, float prev_data)
{
  float output = prev_data + (alfa * (new_data - prev_data));
  return output;
}

float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	I2C_rx_data[I2C_rx_data_index] = I2C_rx_buffer;
	
	
	
	I2C_rx_data_index ++;
	HAL_I2C_Slave_Receive_DMA(&hi2c1, &I2C_rx_buffer, 1);
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
