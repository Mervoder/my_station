/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "lwgps/lwgps.h"
#include <math.h>
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Lora_Rx_Buffer_SIZE 72
#define RX_BUFFER_SIZE 128
#define HYI_BUFFER_SIZE 78

#define TAKIM_ID 31


#define CMD_SET_REG 0xC0 // COMMAND FOR SETTING REGISTER
#define CMD_READ_REG 0xC1 // COMMAND FOR READING REGISTER
#define REG_ADD_H 0x0 // DEVICE ADDRESS HIGH BYTE
#define REG_ADD_L 0x1 // DEVICE ADDRESS LOW BYTE
#define REG0 0x2 // UART CONFIGURATION REGISTER
#define REG1 0x3 // RF CONFIGURATION REGISTER
#define REG2 0x4 // CHANNEL CONTROL
#define REG3 0x5 // TRANSMISSION PARAMETER CONTROL
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
uint8_t Lora_Rx_Buffer[Lora_Rx_Buffer_SIZE],
		HYI_BUFFER[HYI_BUFFER_SIZE];

uint8_t rx_buffer_gps[RX_BUFFER_SIZE]	;

uint8_t rx_data_lora=0;

uint8_t rs_index=0,
		rx_index_lora=0;

uint8_t flag_lora ,
		flag_megu ,
		flag_sensor_imu ,
		flag_sensor_barometre,
		flag_counter,
		flag_adc_cnt,
		flag_adc,
		flag_median;

uint8_t Cmd_End[3] = {0xff,0xff,0xff};
uint8_t nextion_rx_data[5];

const uint8_t EGU_durum_sorgusu[5]={0x54,0x52,0x35,0x0D,0x0A};
const uint8_t EGU_motor_atesleme[5]={0x54,0x52,0x32,0x0D,0x0A};

int time;
float adc , adc_pil_val;

//egu
uint8_t EGU_ARIZA=0;
uint8_t EGU_AYRILMA_TESPIT=0;
uint8_t EGU_MOTOR_ATESLEME_TALEP_IN=0;
uint8_t EGU_STAGE_DURUM=0;
uint8_t EGU_UCUS_BASLADIMI=0;
uint8_t EGU_FITIL =0;

float EGU_BATTERY=0;
float EGU_IRTIFA=0;
float EGU_ANGLE=0;

uint8_t b_altitude[8];
uint8_t b_temperature[5];
uint8_t b_speed[5];
uint8_t b_roll[5];
uint8_t b_pitch[5];
uint8_t b_latitude[9];
uint8_t b_longitude[9];
uint8_t b_bat[2];
uint8_t b_sats[2];
uint8_t b_comm[2];
uint8_t b_dist[6];

uint8_t enum_bs[9];
uint8_t enum_s[9];

char s_altitude[8];
uint8_t s_temperature[5];
uint8_t s_speed[5];
uint8_t s_roll[5];
uint8_t s_pitch[5];
uint8_t s_latitude[9];
uint8_t s_longitude[9];
uint8_t s_bat[2];
uint8_t s_sats[2];
uint8_t s_comm[2];
uint8_t s_dist[6];

///////////////////////////////////////////////////
uint8_t seconds[2];
uint8_t minutes[2];
uint8_t hours[2];
uint8_t st_bat[2];
/////////////////////////////////////////////////
uint8_t p_latitude[9];
uint8_t p_longitude[9];
uint8_t p_altitude[7];
uint8_t p_gpsaltitude[7];
uint8_t p_bat[2];
////////////////////////////////////////////////
uint8_t e_altitude[7];
uint8_t e_bat[2];
uint8_t e_angle[5];
uint8_t e_flight[2];
uint8_t e_stage[2];
uint8_t e_fitil[5];
uint8_t e_engine_request[3];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void union_converter();
void Booster_union_converter();
void Sustainer_union_converter();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Nextion_SendCommand(char* command);
void Nextion_SendFloatToTextbox(char* textbox_id, float value);
void NEXTION_SendString (char *ID, char *string);
void NEXTION_SendNum (char *obj, int32_t num);
void NEXTION_SendFloat (char *obj, float num, int dp);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t E220_read_register(uint8_t reg);
int8_t E220_write_register(uint8_t reg,uint8_t parameter);

float BME280_Get_Altitude(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef union{
  float fVal;
  unsigned char array[4];
}float2unit8;
float2unit8 conv;

typedef struct
{	uint8_t satsinview;
	float gpsaltitude;
	float gpslatitude;
	float gpslongitude;
	float speed;
	float altitude;
	float temperature;
	float accx;
	float accy;
	float accz;
	float normal;
	float pitch;
	float maxAltitude;
	uint8_t battery;
	uint8_t mod;
	uint8_t communication;

}dataTypeDef;

dataTypeDef Payload, Booster , Sustainer;



lwgps_t gps;

int8_t rslt=0 , receive_data;



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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(100);
  receive_data =E220_write_register(0x2, 0x64);
  HAL_Delay(100);

  HAL_Delay(100);
  receive_data =E220_write_register(0x2, 0x64);
  HAL_Delay(100);

  receive_data =E220_write_register(0x3, 0x40);
  HAL_Delay(100);
  receive_data =E220_write_register(0x4, 0x10); // ch
  HAL_Delay(100);
  receive_data =E220_write_register(0x5, 0x40);//40
  HAL_Delay(100);
  receive_data =E220_write_register(0x6, 0x00);
  HAL_Delay(100);
  receive_data =E220_write_register(0x7, 0x00);
  HAL_Delay(100);
  receive_data =E220_write_register(0, 0x06); // h 0x06
  HAL_Delay(100);

  receive_data =E220_write_register(0x1, 0x03); // low 0x03
  HAL_Delay(200);

  receive_data = E220_read_register(0);
  HAL_Delay(100);
  receive_data = E220_read_register(1);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);//m0
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET); //m1
  HAL_Delay(100);

  lwgps_init(&gps);


  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  HAL_ADC_Start_IT(&hadc1);

  //HAL_UART_Receive_IT(&huart3, &rx_data_lora, 1);
  HAL_UART_Receive_DMA(&huart3, Lora_Rx_Buffer, 72);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(Lora_Rx_Buffer[0]==2 && Lora_Rx_Buffer[47] == 0x31){

	  		  Sustainer.satsinview=Lora_Rx_Buffer[1];

	  		  Sustainer_union_converter();
	  		  Sustainer.battery=Lora_Rx_Buffer[46];
	  		  Sustainer.mod=Lora_Rx_Buffer[70];
	  		  Sustainer.communication=Lora_Rx_Buffer[48];

	  			 //EGU PART
	  			 EGU_ARIZA=Lora_Rx_Buffer[49];
	  			 EGU_AYRILMA_TESPIT=Lora_Rx_Buffer[50];

	  			 float2unit8 f2u8_EGU_BATTERY;
	  			for(uint8_t i=0;i<4;i++)
	  			{
	  				f2u8_EGU_BATTERY.array[i]=Lora_Rx_Buffer[i+51];
	  			}
	  			 EGU_BATTERY=f2u8_EGU_BATTERY.fVal;

	  			 float2unit8 f2u8_EGU_ANGLE;
	  			for(uint8_t i=0;i<4;i++)
	  			{
	  				f2u8_EGU_ANGLE.array[i]=Lora_Rx_Buffer[i+55];
	  			}
	  			  EGU_ANGLE=f2u8_EGU_ANGLE.fVal;

	  			  float2unit8 f2u8_EGU_IRTIFA;
	  			for(uint8_t i=0;i<4;i++)
	  			{
	  				f2u8_EGU_IRTIFA.array[i]=Lora_Rx_Buffer[i+59];
	  			}
	  		  EGU_IRTIFA=f2u8_EGU_IRTIFA.fVal;

	  		  EGU_FITIL=Lora_Rx_Buffer[50];
	  //	/*  EGU_UCUS_BASLADIMI*/sustv4_mod=Lora_Rx_Buffer[63];
	  		  EGU_STAGE_DURUM=Lora_Rx_Buffer[64];
	  		  EGU_MOTOR_ATESLEME_TALEP_IN=Lora_Rx_Buffer[65];
	  		  float2unit8 f2u8_altitude;
	  		  f2u8_altitude.array[0] = Lora_Rx_Buffer[66];
	  		  f2u8_altitude.array[1] = Lora_Rx_Buffer[67];
	  		  f2u8_altitude.array[2] = Lora_Rx_Buffer[68];
	  		  f2u8_altitude.array[3] = Lora_Rx_Buffer[69];
	  		  Sustainer.maxAltitude=f2u8_altitude.fVal;



				sprintf(s_altitude,"%4.3f",Sustainer.altitude);
				sprintf(s_temperature,"%2.2f",Sustainer.temperature);
				sprintf(s_speed,"%2.2f",Sustainer.speed);
				sprintf(s_roll,"%2.2f",Sustainer.normal);
				sprintf(s_pitch,"%2.2f",Sustainer.pitch);
				sprintf(s_latitude,"%2.6f",Sustainer.gpslatitude);
				sprintf(s_longitude,"%2.6f",Sustainer.gpslongitude);
				sprintf(s_bat,"%2d",Sustainer.battery);
				sprintf(s_sats,"%2d",Sustainer.satsinview);
				sprintf(s_comm,"%2d",Sustainer.communication);
				sprintf(s_comm,"%2d",Sustainer.communication);
				//sprintf(s_dist,"%4.2f",s_distance);

				sprintf(e_altitude,"%4.2f",EGU_IRTIFA);
				sprintf(e_angle,"%2.2f",EGU_ANGLE);
				sprintf(e_bat,"%2d",EGU_BATTERY);
				sprintf(e_flight,"%d",EGU_UCUS_BASLADIMI);
				sprintf(e_stage,"%d",EGU_AYRILMA_TESPIT);
				sprintf(e_fitil,"%d",EGU_FITIL);
				// Nextion_SendFloatToTextbox("s1", Sustainer.altitude);
				 NEXTION_SendString("s1", s_altitude);
				 NEXTION_SendString("s2", s_temperature);
				 NEXTION_SendString("s3", s_speed);
				 NEXTION_SendString("s4", s_roll);
				 NEXTION_SendString("s5", s_pitch);
				 NEXTION_SendString("s6", s_sats);
				 NEXTION_SendString("s7", s_latitude);
				 NEXTION_SendString("s8", s_longitude);
				 NEXTION_SendString("s9", s_bat);
				 NEXTION_SendString("t57", s_comm);
				 NEXTION_SendString("t", s_dist);


				 NEXTION_SendString("m1", e_altitude);
				 NEXTION_SendString("m3", e_angle);
				 NEXTION_SendString("m2", e_bat);
				 NEXTION_SendString("m4", e_flight);
				 NEXTION_SendString("m5", e_stage);


	  }



	  if(Lora_Rx_Buffer[0]==1 && Lora_Rx_Buffer[47]==0x32){

	  		  Booster.satsinview=Lora_Rx_Buffer[1];

	      	  Booster_union_converter();
	  		  Booster.battery=Lora_Rx_Buffer[46];
	  		  Booster.mod=Lora_Rx_Buffer[70];
	  		  Booster.communication=Lora_Rx_Buffer[48];
	  		  float2unit8 f2u8_booster;
	  		  f2u8_booster.array[0] = Lora_Rx_Buffer[66];
	  		  f2u8_booster.array[1] = Lora_Rx_Buffer[67];
	  		  f2u8_booster.array[2] = Lora_Rx_Buffer[68];
	  		  f2u8_booster.array[3] = Lora_Rx_Buffer[69];
	  		  Booster.maxAltitude = f2u8_booster.fVal;


	  		sprintf(b_altitude,"%4.3f",Booster.altitude);
			sprintf(b_temperature,"%2.2f",Booster.temperature);
			sprintf(b_speed,"%2.2f",Booster.speed);
			sprintf(b_roll,"%2.2f",Booster.normal);
			sprintf(b_pitch,"%2.2f",Booster.pitch);
			sprintf(b_latitude,"%2.6f",Booster.gpslatitude);
			sprintf(b_longitude,"%2.6f",Booster.gpslongitude);
			sprintf(b_bat,"%2d",Booster.battery);
			sprintf(b_sats,"%2d",Booster.satsinview);
			sprintf(b_comm,"%2d",Booster.communication);
		//	sprintf(b_dist,"%4.2f",bs_distance);

			NEXTION_SendString("bs1", b_altitude);
			NEXTION_SendString("bs2", b_temperature);
			NEXTION_SendString("bs3", b_speed);
			NEXTION_SendString("bs4", b_roll);
			NEXTION_SendString("bs5", b_pitch);
			NEXTION_SendString("bs6", b_sats);
			NEXTION_SendString("bs7", b_latitude);
			NEXTION_SendString("bs8", b_longitude);
			NEXTION_SendString("bs9", b_bat);
			NEXTION_SendString("t56", b_comm);
			NEXTION_SendString("t17", b_dist);

			NEXTION_SendString("m7", e_engine_request);
			NEXTION_SendString("t59", e_fitil);


//		 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
//
//			HAL_Delay(3);
//
//			uint8_t send_data[4]={0xC2,0x0,1,0x06};
//			uint8_t receive_data[4]={0};
//
//			HAL_UART_Transmit(&huart3,send_data ,4, 100);
//			HAL_UART_Receive(&huart3, receive_data, 4, 100);
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);

	  }

	  if(flag_adc_cnt >=10 && flag_adc ==1)
	  	  {
	  		  if(adc > 2476) adc = 2234;
	  		  if(adc < 1755) adc = 1755;
	  		  // 6V = 1755 adc val 1,41V
	  		  // 8.4V = 2476 adc val 1,99V 0,58V
	  		  adc_pil_val=(float)( ( ( (adc/4095)*3.3)-1.41) / (1.99-1.41) ) *100 ;
	  		 // adc_pil_val = (adc-1755)/(2746-1755)*100;

	      	sprintf(st_bat,"%2d",(uint8_t)adc_pil_val);
	      	NEXTION_SendString("t54", st_bat);

	  		flag_adc=0;
	  		flag_adc_cnt=0;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  htim2.Init.Prescaler = 8400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 16000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8400;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 300-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8400;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 19200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|CS_Pin|Buzzer_Pin|GATE_D_Pin
                          |GATE_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M0_Pin|M1_Pin|FN_Pin|Led2_Pin
                          |Led1_Pin|GATE_B_Pin|GATE_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 CS_Pin Buzzer_Pin GATE_D_Pin
                           GATE_C_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|CS_Pin|Buzzer_Pin|GATE_D_Pin
                          |GATE_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 Button_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_14|Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M0_Pin M1_Pin FN_Pin Led2_Pin
                           Led1_Pin GATE_B_Pin GATE_A_Pin */
  GPIO_InitStruct.Pin = M0_Pin|M1_Pin|FN_Pin|Led2_Pin
                          |Led1_Pin|GATE_B_Pin|GATE_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim==&htim2)// Lora timer
	{
		flag_lora=1;

		flag_adc_cnt++;
		if(flag_adc_cnt >=11) flag_adc_cnt=0;
	}

	if(htim==&htim3)// sensor timer 30ms
	{
		flag_sensor_imu=1;

		flag_counter++;
		if(flag_counter == 10)
		{
			flag_sensor_barometre =1;
			flag_counter=0;
		}
	}

	if(htim==&htim4)// megü timer
	{
		flag_megu=1;

	}

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	if(huart==&huart2){
//	if(rx_data_gps != '\n' && rx_index_gps < RX_BUFFER_SIZE) {
//		rx_buffer_gps[rx_index_gps++] = rx_data_gps;
//	} else {
//		lwgps_process(&gps, rx_buffer_gps, rx_index_gps+1);
//		rx_index_gps = 0;
//		rx_data_gps = 0;
//	}
//	HAL_UART_Receive_IT(&huart2, &rx_data_gps, 1);
//	}


	if(huart == &huart3){
//		if(rx_data_lora != '\n'&& rx_index_lora < Lora_Rx_Buffer_SIZE){
//			Lora_Rx_Buffer[rx_index_lora++]=rx_data_lora;
//
//		}
//		else{
//			rx_data_lora=0;
//			rx_index_lora=0;
//
//			}
//		HAL_UART_Receive_IT(&huart3, &rx_data_lora, 1);

		 HAL_UART_Receive_DMA(&huart3, Lora_Rx_Buffer, 72);
		}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1 )
	{
		adc= HAL_ADC_GetValue(&hadc1);
		flag_adc=1;
	}
}

int8_t E220_write_register(uint8_t reg,uint8_t parameter)
{

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);

	HAL_Delay(3);

	uint8_t send_data[4]={CMD_SET_REG,reg,1,parameter};
	uint8_t receive_data[4]={0};

	HAL_UART_Transmit(&huart3,send_data ,4, 100);
	HAL_UART_Receive(&huart3, receive_data, 4, 100);


	if(receive_data[0]==CMD_READ_REG && receive_data[1]==reg && receive_data[2]==1 && receive_data[3] == parameter)
		return receive_data[3];
	else
		return -1;

}
int8_t E220_read_register(uint8_t reg)
{


	uint8_t send_data[3]={CMD_READ_REG,reg,1};
	uint8_t receive_data[4]={0};
	HAL_UART_Transmit(&huart3,send_data ,3, 100);



	HAL_UART_Receive(&huart3, receive_data, 4, 100);

	if(receive_data[0]==CMD_READ_REG && receive_data[1]==reg && receive_data[2]==1)
		return receive_data[3];
	else
		return -1;
}


void NEXTION_SendString (char *ID, char *string)
{
	char buf[50];
	int len = sprintf (buf, "%s.txt=\"%s\"", ID, string);
	HAL_UART_Transmit(&huart4, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);
}


void NEXTION_SendNum (char *obj, int32_t num)
{
	uint8_t *buffer = malloc(30*sizeof (char));
	int len = sprintf ((char *)buffer, "%s.val=%ld", obj, num);
	HAL_UART_Transmit(&huart4, buffer, len, 1000);
	HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);
	free(buffer);
}


void NEXTION_SendFloat (char *obj, float num, int dp)
{
	// convert to the integer
	int32_t number = num*(pow(10,dp));

	uint8_t *buffer = malloc(30*sizeof (char));
	int len = sprintf ((char *)buffer, "%s.vvs1=%d", obj, dp);
	HAL_UART_Transmit(&huart4, buffer, len, 1000);
	HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);


	len = sprintf ((char *)buffer, "%s.val=%ld", obj, number);
	HAL_UART_Transmit(&huart4, buffer, len, 1000);
	HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);
	free(buffer);
}

void Nextion_SendCommand(char* command) {
    HAL_UART_Transmit(&huart4, (uint8_t*)command, strlen(command), 100);
    uint8_t end_cmd[] = {0xFF, 0xFF, 0xFF}; // Nextion end of command
    HAL_UART_Transmit(&huart4, end_cmd, 3, 100);
}

// Function to send a float value to a Nextion text box
void Nextion_SendFloatToTextbox(char* textbox_id, float value) {
    char command[50];
    char value_str[20];

    // Convert the float to a string
    snprintf(value_str, sizeof(value_str), "%.2f", value); // Adjust the format as needed

    // Format the command
    snprintf(command, sizeof(command), "%s.txt=\"%s\"", textbox_id, value_str);

    // Send the command to the Nextion display
    Nextion_SendCommand(command);
}



void Booster_union_converter(void)
{
			float2unit8 f2u8_booster;
					 for(uint8_t i=0;i<4;i++)
					 {
						 f2u8_booster.array[i]=Lora_Rx_Buffer[i+2];
						 HYI_BUFFER[34+i]=Lora_Rx_Buffer[i+5]; // 34 35 36 37
					 }
					 Booster.gpsaltitude=f2u8_booster.fVal;


					 for(uint8_t i=0;i<4;i++)
					 {
						 f2u8_booster.array[i]=Lora_Rx_Buffer[i+6];
						 HYI_BUFFER[38+i]=Lora_Rx_Buffer[i+9]; // 38 39 40 41
					 }
					 Booster.gpslatitude=f2u8_booster.fVal;

					 for(uint8_t i=0;i<4;i++)
					 {
						 f2u8_booster.array[i]=Lora_Rx_Buffer[i+10];
						 HYI_BUFFER[42+i]=Lora_Rx_Buffer[i+13]; // 42 43 44 45
					 }
					 Booster.gpslongitude=f2u8_booster.fVal;

					 for(uint8_t i=0;i<4;i++)
					 {
						 f2u8_booster.array[i]=Lora_Rx_Buffer[i+14];
					 }
					 Booster.altitude=f2u8_booster.fVal;



					 for(uint8_t i=0;i<4;i++)
					 {
						 f2u8_booster.array[i]=Lora_Rx_Buffer[i+18];
					 }
					 Booster.speed=f2u8_booster.fVal;


					 for(uint8_t i=0;i<4;i++)
					 {
						 f2u8_booster.array[i]=Lora_Rx_Buffer[i+22];
					 }
					 Booster.temperature=f2u8_booster.fVal;


					 for(uint8_t i=0;i<4;i++)
					 {
						 f2u8_booster.array[i]=Lora_Rx_Buffer[i+26];
					 }
					 Booster.accx=f2u8_booster.fVal;


					 for(uint8_t i=0;i<4;i++)
					 {
						 f2u8_booster.array[i]=Lora_Rx_Buffer[i+30];
					 }
					 Booster.accy=f2u8_booster.fVal;


				      for(uint8_t i=0;i<4;i++)
					 {
				    	  f2u8_booster.array[i]=Lora_Rx_Buffer[i+34];
					 }
				      Booster.accz=f2u8_booster.fVal;

					  for(uint8_t i=0;i<4;i++)
					 {
						  f2u8_booster.array[i]=Lora_Rx_Buffer[i+38];
					 }
					  Booster.normal=f2u8_booster.fVal;

					  for(uint8_t i=0;i<4;i++)
					 {
						  f2u8_booster.array[i]=Lora_Rx_Buffer[i+42];
					 }
					  Booster.pitch=f2u8_booster.fVal;
}




void Sustainer_union_converter(void)
{
	 float2unit8 f2u8_gpsalt;
					 for(uint8_t i=0;i<4;i++)
					 {
						 f2u8_gpsalt.array[i]=Lora_Rx_Buffer[i+2];
						 HYI_BUFFER[10+i] =Lora_Rx_Buffer[i+5]; // 10 11 12 13
					 }
					 Sustainer.gpsaltitude=f2u8_gpsalt.fVal;
				 float2unit8 f2u8_latitude;

					 for(uint8_t i=0;i<4;i++)
					 {
						f2u8_latitude.array[i]=Lora_Rx_Buffer[i+6];
						HYI_BUFFER[14+i] =Lora_Rx_Buffer[i+9]; // 14 15 16 17
					 }
					 Sustainer.gpslatitude=f2u8_latitude.fVal;

				 float2unit8 f2u8_longitude;
					 for(uint8_t i=0;i<4;i++)
					 {
						f2u8_longitude.array[i]=Lora_Rx_Buffer[i+10];
						HYI_BUFFER[18+i] =Lora_Rx_Buffer[i+13]; // 18 19 20 21
					 }
					 Sustainer.gpslongitude=f2u8_longitude.fVal;

				 float2unit8 f2u8_altitude;
					 for(uint8_t i=0;i<4;i++)
					 {
						f2u8_altitude.array[i]=Lora_Rx_Buffer[i+14];
						HYI_BUFFER[6+i] =Lora_Rx_Buffer[i+17]; // 6 7 8 9
					 }
					 Sustainer.altitude=f2u8_altitude.fVal;

				 float2unit8 f2u8_speed;

					 for(uint8_t i=0;i<4;i++)
					 {
						 f2u8_speed.array[i]=Lora_Rx_Buffer[i+18];
					 }
					 Sustainer.speed=f2u8_speed.fVal;

				 float2unit8 f2u8_temp;
					 for(uint8_t i=0;i<4;i++)
					 {
						 f2u8_temp.array[i]=Lora_Rx_Buffer[i+22];
					 }
					 Sustainer.temperature=f2u8_temp.fVal;

				 float2unit8 f2u8_accx;
					 for(uint8_t i=0;i<4;i++)
					 {
						 f2u8_accx.array[i]=Lora_Rx_Buffer[i+26];
						// HYI_BUFFER[58+i]=Lora_Rx_Buffer[i+29]; //
					 }
					 Sustainer.accx=f2u8_accx.fVal;

				float2unit8 f2u8_accy;
					 for(uint8_t i=0;i<4;i++)
					 {
						 f2u8_accy.array[i]=Lora_Rx_Buffer[i+30];
						 //HYI_BUFFER[62+i]=Lora_Rx_Buffer[i+33];
					 }
					 Sustainer.accy=f2u8_accy.fVal;

				float2unit8 f2u8_accz;
				      for(uint8_t i=0;i<4;i++)
					 {
				    	  f2u8_accz.array[i]=Lora_Rx_Buffer[i+34];
				    	//  HYI_BUFFER[66+i]=Lora_Rx_Buffer[i+37];
					 }
				      Sustainer.accz=f2u8_accz.fVal;

				float2unit8 f2u8_roll;
					  for(uint8_t i=0;i<4;i++)
					 {
						  f2u8_roll.array[i]=Lora_Rx_Buffer[i+38];
					 }
					  Sustainer.normal=f2u8_roll.fVal;

				float2unit8 f2u8_pitch;
					  for(uint8_t i=0;i<4;i++)
					 {
						  f2u8_pitch.array[i]=Lora_Rx_Buffer[i+42];
					 }
					  Sustainer.pitch=f2u8_pitch.fVal;

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
