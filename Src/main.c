/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usbd_customhid.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

osThreadId DefaultTaskHandle;
osThreadId UARTTaskHandle;
osThreadId USBTaskHandle;
osThreadId SensorTaskHandle;
osThreadId FFBControlTaskHandle;

osThreadId ImmediateAnswerTaskHandle;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern uint32_t uwTick;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

static const uint16_t inputAxis[10]     = {0,  5, 10, 20, 30, 50, 80, 120, 170, 255};
//static const uint16_t inputAxis[10]     = {0, 5, 15, 40, 70, 100, 150, 170, 190, 255};

//static const uint16_t feedbackCurve[10] = {0, 15, 27, 39, 48, 60, 72, 82,  90,  100}; // sportos
static const uint16_t feedbackCurve[10] = {0, 15, 20, 27, 32, 40, 50, 63,  75,  90};  // komfort
//static const uint16_t feedbackCurve[10] = {0, 15, 60, 90, 30, 85, 80, 30, 60, 30};  // komfort

static uint16_t testInput = 0;
static volatile uint16_t testOutput = 0;

__IO uint16_t pedalsADC[2];

uint8_t charBuffer;
#define INPUTLENGTH 10
uint8_t ReceivedString[INPUTLENGTH+1];
uint8_t ReceivedFlag = 0;

uint8_t buffer[7];
uint8_t oldBuffer[7];
uint8_t buffer2[3];

static uint16_t errorCounter = 0;

static int16_t gas = 0;
static int16_t brake = 0;
static uint16_t gasMin = 10000;
static uint16_t brakeMin = 10000;
static uint16_t gasMax = 0;
static uint16_t brakeMax = 0;
static volatile float gasFactor = 1.0;
static volatile float brakeFactor = 1.0;

static volatile int16_t rawAngle = 0;
static volatile float stwa_deg = 0;
static volatile float stwasp_deg = 0;
static volatile float stwacc_deg = 0;

static float stwa_deg_prev = 0;
static float stwasp_deg_prev = 0;

static volatile float stwasp_deg_filt = 0;
static volatile float stwacc_deg_filt = 0;

static int16_t force = 0;
static int16_t damForce = 0;
static int16_t absForce = 0;
static float spring = 0.05;
//static float stwFactor = 7.0;

static int16_t stwaOut = 0;
static float stwaMax = 135.0; // max half turn angle

volatile uint8_t constantForceEna = 0;
uint32_t cfStart = 0;
uint16_t cfDelay = 0;
uint8_t  cfDirection = 0;
uint16_t cfDuration = 0;
int16_t  cfConstant = 0;
int16_t  cfConstantNorm = 0;
float dirGain = 0;

uint8_t AnswerFlag = 0;
uint8_t sendBusy = 0;
uint8_t ReceiveFlag = 0;

static uint8_t defaultSpringEna = 0; //SPRING
static uint8_t endLockEna = 1;       //ELP
static uint8_t damEna = 0;

uint32_t USBReceiveTime = 0;

uint32_t lastMessageTime = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartUARTTask(void const * argument);
void StartUSBTask(void const * argument);
void StartSensorTask(void const * argument);
void StartFFBControlTask(void const * argument);

void StartImmediateAnswerTask(void const * argument);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
    /* with GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
       set to 'Yes') calls __io_putchar() */
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint16_t calculateFBC_LUT(int16_t inForce){
  uint16_t outForce = 0;
  uint8_t i;
  
  for (i = 0; i < 10; i++) {
    if (inForce <= inputAxis[i]) {
      if (inForce == inputAxis[i]) {
        outForce = feedbackCurve[i];
        break;
      }
      else {
        outForce = feedbackCurve[i-1] + ((feedbackCurve[i] - feedbackCurve[i-1])*(inForce-inputAxis[i-1]))/(inputAxis[i]-inputAxis[i-1]);
        break;
      }
    }
    else {
      outForce = feedbackCurve[9];
    }
  }
  
  return outForce;
}


void InitUARTInterrupt(UART_HandleTypeDef *huart){
  
  HAL_UART_Receive_IT(huart, &charBuffer, 1);
}
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
  MX_ADC1_Init();
  MX_RNG_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  

  /* USER CODE BEGIN 2 */
  
  //HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  
  InitUARTInterrupt(&huart2);
  
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_2);
  
  
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
  
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)pedalsADC, 2);
  
  buffer[0] = 0x01;
  buffer[1] = 0x00;
  buffer[2] = 0x00;
  buffer[3] = 0x00;
  buffer[4] = 0x00;
  buffer[5] = 0x00;
  buffer[6] = 0x00;
  
  oldBuffer[0] = 0x01;
  oldBuffer[1] = 0x00;
  oldBuffer[2] = 0x00;
  oldBuffer[3] = 0x00;
  oldBuffer[4] = 0x00;
  oldBuffer[5] = 0x00;
  oldBuffer[6] = 0x00;
  
  buffer2[0] = 0x02;
  buffer2[1] = 0x32;
  buffer2[2] = 0x05;
  
  USBReceiveTime = uwTick;

  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
  //TIM4->CCR1 = 50*16800/100;

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of DefaultTask */
  osThreadDef(DefaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  DefaultTaskHandle = osThreadCreate(osThread(DefaultTask), NULL);

  /* definition and creation of UARTTask */
  osThreadDef(UARTTask, StartUARTTask, osPriorityLow, 0, 128);
  UARTTaskHandle = osThreadCreate(osThread(UARTTask), NULL);

  /* definition and creation of USBTask */
  osThreadDef(USBTask, StartUSBTask, osPriorityNormal, 0, 128);
  USBTaskHandle = osThreadCreate(osThread(USBTask), NULL);

  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, StartSensorTask, osPriorityAboveNormal, 0, 128);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* definition and creation of FFBControlTask */
  osThreadDef(FFBControlTask, StartFFBControlTask, osPriorityHigh, 0, 128);
  FFBControlTaskHandle = osThreadCreate(osThread(FFBControlTask), NULL);

  osThreadDef(ImmediateAnswerTask, StartImmediateAnswerTask, osPriorityRealtime, 0, 128);
  ImmediateAnswerTaskHandle = osThreadCreate(osThread(ImmediateAnswerTask), NULL);
  
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* RNG init function */
void MX_RNG_Init(void)
{

  hrng.Instance = RNG;
  HAL_RNG_Init(&hrng);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  HAL_TIM_Encoder_Init(&htim1, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;//32
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4200;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
   * @brief Retargets the C library printf function to the USART
   * @param  None
   * @retval None
   */
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    
    return ch;
}

void saturateInteger(int16_t* i, int16_t min, int16_t max) {
  int16_t val = *i;
  if (val < min) val = min;
  else if (val > max) val = max;
  *i = val;
}

int16_t calculateSpringForce(float stwa, float stwa_lim, float stwa_offset, float stwa_breakpoint, int16_t force_breakpoint, int16_t force_lim){

  float stiffnessUnderBreakpoint = force_breakpoint / stwa_breakpoint;
  float stiffnessOverBreakpoint  = (force_lim - force_breakpoint) / (stwa_lim - stwa_breakpoint);
  float calcForce = 0;
  int16_t intForce = 0;
  
  float M = 0.0;
  
  if (((stwa + stwa_offset) >= 0) && (stwa + stwa_offset) < (stwa_breakpoint)){
    calcForce = (stwa + stwa_offset) * stiffnessUnderBreakpoint + stwacc_deg_filt*M;
  }
  else if (((stwa + stwa_offset) < 0) && (stwa + stwa_offset) > (-stwa_breakpoint)){
    calcForce = (stwa + stwa_offset) * stiffnessUnderBreakpoint + stwacc_deg_filt*M;
  }
  else if ((stwa + stwa_offset) >= (stwa_breakpoint)) {
    calcForce = force_breakpoint + (stwa - stwa_breakpoint + stwa_offset) * stiffnessOverBreakpoint + stwacc_deg_filt*M;
  }
  else if ((stwa + stwa_offset) <= (-stwa_breakpoint)) {
    calcForce = -force_breakpoint + (stwa + stwa_offset + stwa_breakpoint) * stiffnessOverBreakpoint + stwacc_deg_filt*M;
  }
  else {
    calcForce = 0;
  }
  
  intForce = (int16_t)calcForce;
  saturateInteger(&intForce, -force_lim, force_lim);
  
  return intForce;
  
}

float filter1 (float avg, float input, int alpha) {
    avg -= avg/alpha;
    avg += input/alpha;
    return avg;
}

float filter2 (float avg, float input, float alpha) {
  avg = (alpha * input) + (1.0 - alpha) * avg;
  return avg;
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  USB_OTG_GlobalTypeDef *USBx = (&hpcd_USB_OTG_FS)->Instance;
  USBD_CUSTOM_HID_HandleTypeDef     *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)(&hUsbDeviceFS)->pClassData;
  /* USER CODE BEGIN 5 */
  
  /* Infinite loop */
  for(;;)
  {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {

      USBD_LL_PrepareReceive(&hUsbDeviceFS, CUSTOM_HID_EPOUT_ADDR , hhid->Report_buf, 
                         USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);
    }
    if ((sendBusy) && ((lastMessageTime + 100) < uwTick)){
      errorCounter++;
      USBD_LL_PrepareReceive(&hUsbDeviceFS, CUSTOM_HID_EPOUT_ADDR , hhid->Report_buf, 
                         USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);
    }
    
    //testInput = testInput%300;
    //testOutput = 0;
    //testOutput = calculateFBC_LUT(testInput);
    //testInput++;

    osDelay(100);
  }
  /* USER CODE END 5 */ 
}

/* StartUARTTask function */
void StartUARTTask(void const * argument)
{
  /* USER CODE BEGIN StartUARTTask */
  uint8_t LastReceivedFlag = ReceivedFlag;
  /* Infinite loop */
  for(;;)
  {
    if (ReceivedFlag != LastReceivedFlag){
      LastReceivedFlag = ReceivedFlag;
      printf("echo: %s\r\n", ReceivedString);
    }
    osDelay(100);
  }
  /* USER CODE END StartUARTTask */
}

/* StartUSBTask function */
void StartUSBTask(void const * argument)
{
  /* USER CODE BEGIN StartUSBTask */
  USBD_CUSTOM_HID_HandleTypeDef     *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)(&hUsbDeviceFS)->pClassData;
  uint8_t success = 1;
  
  
  uint8_t lastBuffer = 0;
  uint8_t changeFlag = 1;
  uint8_t i;
  
  osDelay(10);
  /* Infinite loop */
  for(;;)
  {
    
    stwaOut = (int)(stwa_deg * (2047.0/stwaMax));
    saturateInteger(&stwaOut, -2047, 2047);
    stwaOut += 2047;
      
    buffer[3] = stwaOut & 0xFF;
    buffer[4] = (stwaOut & 0xFF00) >> 8;
    buffer[5] = gas;
    buffer[6] = brake;
    
    //if (FFBFlag){
    //  osDelay(10);
    //  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,buffer2,3);
    //  FFBFlag = 0;
    //}
    
    //buffer[3] = 1;
    //lastBuffer = 0;
    
    changeFlag = 0;
    for (i = 0; i < 7; i++) {
      if (buffer[i] != oldBuffer[i])changeFlag = 1;
    }
    
    //oldBuffer[1] = 0;
    //buffer[1] = 1;
    //changeFlag = 1;
    
    if ((changeFlag) && ((USBReceiveTime + 5) < uwTick)){
      if (sendBusy == 0) success = USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,buffer,7);
      oldBuffer[0] = buffer[0];
      oldBuffer[1] = buffer[1];
      oldBuffer[2] = buffer[2];
      oldBuffer[3] = buffer[3];
      oldBuffer[4] = buffer[4];
      oldBuffer[5] = buffer[5];
      oldBuffer[6] = buffer[6];

      if (success != 0) {
        __asm("NOP");
      }
    }

    osDelay(10);

  }
  /* USER CODE END StartUSBTask */
}

/* StartSensorTask function */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */

  /* Infinite loop */
  for(;;)
  {
    rawAngle = TIM1->CNT;
    stwa_deg_prev = stwa_deg;
    stwasp_deg_prev = stwasp_deg_filt;
      
    stwa_deg = rawAngle * 0.239;
    stwasp_deg = (stwa_deg - stwa_deg_prev) * 100;
    stwasp_deg_filt = filter2(stwasp_deg_filt, stwasp_deg, 0.3);
    stwacc_deg = (stwasp_deg_filt - stwasp_deg_prev) * 100;
    stwacc_deg_filt = filter2(stwacc_deg_filt, stwacc_deg, 0.05);
    
    
    if (pedalsADC[0] > gasMax) gasMax = pedalsADC[0];
    if (pedalsADC[0] < gasMin) gasMin = pedalsADC[0];
    if (pedalsADC[1] > brakeMax) brakeMax = pedalsADC[1];
    if (pedalsADC[1] < brakeMin) brakeMin = pedalsADC[1];

    
    if ((gasMin < 1500) && (brakeMin < 1500)){
      
      gasFactor = 4096.0 / (gasMax - gasMin);
      brakeFactor = 4096.0 / (brakeMax - brakeMin);
      
      gas = (int)(4096 - ((pedalsADC[0] - gasMin) * gasFactor)) / 16;
      brake = (int)(4096 - ((pedalsADC[1] - brakeMin) * brakeFactor)) / 16;
      
      if (gas < 60)    gas = 60;
      if (brake < 60)  brake = 60;
      if (gas > 200)   gas = 200;
      if (brake > 200) brake = 200;
      
      gas = (int)((gas - 60)*1.85);
      brake = (int)((brake - 60)*1.85);
    }
    
    saturateInteger(&gas, 0, 255);
    saturateInteger(&brake, 0, 255);
    
    osDelay(10);
  }
  /* USER CODE END StartSensorTask */
}

/* StartFFBControlTask function */
void StartFFBControlTask(void const * argument)
{
  /* USER CODE BEGIN StartFFBControlTask */
  /* Infinite loop */
  for(;;)
  {
    force = 0;
    if (defaultSpringEna) {
      force += calculateSpringForce(stwa_deg, stwaMax, 0.0, 2.0, 20, 60);
    }
    
    if (constantForceEna) {
      if (uwTick > cfStart + cfDelay) {
        
        if ((cfDirection >= 0) && (cfDirection <= 63)){
          dirGain = (cfDirection) / 63.0;
        }
        else if ((cfDirection > 63) && (cfDirection <= 127)){
          dirGain = (127 - cfDirection) / 63.0;
        }
        else if ((cfDirection > 127) && (cfDirection <= 191)){
          dirGain = (cfDirection - 128) / -63.0;
        }
        else if ((cfDirection > 191) && (cfDirection <= 255)){
          dirGain = (255 - cfDirection) / -63.0;
        }
        else {
          __asm("NOP");
        }
        
        /*
        if (cfConstant >= 0) {
          cfConstantNorm = 50 + (cfConstant*50)/255;
        }
        else {
          cfConstantNorm = -50 - (cfConstant*50)/255;
        }
        */
        cfConstantNorm = calculateFBC_LUT(cfConstant);//(cfConstant*150)/255;
          
        force += (int16_t)(cfConstantNorm * dirGain);
      }
      if (uwTick > cfStart + cfDelay + cfDuration){
        constantForceEna = 0;
      }
    }
    
    if (damEna){
      damForce = (int16_t)(stwasp_deg_filt*-0.9);
      saturateInteger(&damForce, -15, 15);
      force += damForce;
    }
    
    if (endLockEna){
      if (stwa_deg > stwaMax){
        force = (int16_t)((stwa_deg - stwaMax)*10);// + stwasp_deg_filt * 0.1);
      }
      else if (stwa_deg < -stwaMax){
        force = (int16_t)((stwa_deg + stwaMax)*10);// + stwasp_deg_filt * 0.1);
      }
    }
    
    //force = 20;
    
    ////// PWM OUT //////
    saturateInteger(&force, -100, 100);
    absForce = abs(force);
    
    if (force > 0) {
      //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
      TIM4->CCR1 = 0;
      TIM4->CCR2 = absForce*4200/100;
    }
    else if (force < 0) {
      //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
      TIM4->CCR1 = absForce*4200/100;
      TIM4->CCR2 = 0;
    }
    else {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
      TIM4->CCR1 = 0;
      TIM4->CCR2 = 0;
    }
    
    //saturateInteger(&absForce, 51, 100);
    //TIM4->CCR1 = absForce*4200/100;
    
    
    osDelay(10);
  }
  /* USER CODE END StartFFBControlTask */
}

void StartImmediateAnswerTask(void const * argument)
{
  /* USER CODE BEGIN StartFFBControlTask */
  uint8_t success = 1;
  //osDelay(5);
  /* Infinite loop */
  for(;;)
  {
    
    if (AnswerFlag) {
      AnswerFlag = 0;
      sendBusy = 1;
      //USB_UNMASK_INTERRUPT((&hpcd_USB_OTG_FS)->Instance, USB_OTG_GINTSTS_IEPINT);

      while(1){
        success = USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,buffer2,3);
        if (success != 0) {
          __asm("NOP");
        }
        else break;
      }
      
      //osDelay(2);
      sendBusy = 0;
      //osDelay(2);
    }
    
    osDelay(1);
  }
}

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
