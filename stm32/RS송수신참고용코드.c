/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "circular_queue.h"
#include "eyetv.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TEST_MODE      0


#define LED_DISPLAY_S   3

#if TEST_MODE == 1
#define SPEED_MIN       10.0             // 
#else
#define SPEED_MIN       10.0             // 동작시
#endif


#define DIR_IN          1
#define DIR_OUT         2

#define M30_STX      0x02
#define M30_ETX      0x03
#define M30_TYPE_TXT 0x84   // "TXT=..."
#define M30_TYPE_CMD 0x85   // "IDX=..."

#define COLOR_RED    1
#define COLOR_GREEN  3
#define COLOR_YELLOW 2


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;


QueueType q1, q2;


char Dp_buff[100];
uint8_t dir;


char str[100];

uint8_t T_f;
uint8_t T_s;
uint8_t T_100ms;
uint8_t T_blink;

uint8_t t100ms;

uint8_t _len;

int sp = 0;
int delay_cnt = 0;

//변수형 변경
volatile uint16_t Speed = 0;   // <-- 네 선언부에서 이 한 줄만 바꾸면 끝
uint16_t old_Speed = 0;
uint16_t Speed_Limit;
uint8_t Color;

//카운터 과속 현재속도 깜빡임
uint8_t Speed_cnt = 0;
uint8_t Speed_cnt_2 = 0;

uint8_t Eye_Update_f;
uint8_t LED_s;
uint8_t Eye_Display_cnt;

uint8_t Speed_Update_f;

uint16_t Speedbuff[500];
uint16_t SpeedCnt;
uint16_t Speed_Avr_s;
uint8_t Speed_Avr_f;


uint8_t Speed_revf;
uint8_t Speed_500m;


uint8_t Init_s;
uint8_t Init_f = 0;


uint8_t update_s;
uint8_t Rey_s;

int cnt_dis = 0;
int ck_cnt = 0;
int delay_cnt_2 = 0;
unsigned long cnt_on = 0;

int speed_cnt_3 = 0;
int tmp_speed = 0;
//int test_sp = 40;

static void WaitMs(uint32_t ms);

// 100ms 틱 기준으로 1.0초씩
#define NUM_TICKS  10
#define TXT_TICKS  10

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM14_Init(void);
static void MX_ADC_Init(void);

/* USER CODE BEGIN PFP */

uint8_t Doppler_read(uint8_t data);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 100);
  return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim->Instance == TIM6)
  {
    T_f = 1;
    
    
    if (Init_s > 0) Init_s--;
    if (T_s > 0) T_s--;
    if (LED_s > 0) LED_s--;
    if (Rey_s > 0) Rey_s--;
  }
  
  if (htim->Instance == TIM14)
  {
    
    if (T_100ms > 0) T_100ms--;
    if (T_blink > 0) T_blink--;
    
    
    if (t100ms++ > 7)
    {
      t100ms = 0;
      Eye_Update_f = 1;
      
    }
    
  }
  
  if (htim->Instance == TIM17)
  {
    if (Speed_500m > 0) Speed_500m--;
    
    
  }
  
}

void Usart_RxLoop(void)
{
  uint8_t data;
  
  
  
  if (is_empty(&q1) != 1)
  {
    data = dequeue(&q1);
    
    
    
  }
  
  
  if (is_empty(&q2) != 1)
  {
    data = dequeue(&q2);
    
    
    Doppler_read(data);
    
    
    
  }
  
  
  
  
  
}

uint16_t SpeedMax(uint16_t* buff, uint16_t len)
{
  uint16_t i;
  uint16_t max;
  
  max = buff[0];
  
  for (i = 0; i < len; i++)
  {
    if (buff[i] > max) max = buff[i];
  }
  
  return max;
}



unsigned int asctohex(const char* str, size_t size, uint8_t* hex)
{
  unsigned int i, h, high, low;
  for (h = 0, i = 0; i < size; i += 2, ++h) {
    
    high = (str[i] > '9') ? str[i] - 'A' + 10 : str[i] - '0';
    low = (str[i + 1] > '9') ? str[i + 1] - 'A' + 10 : str[i + 1] - '0';
    
    hex[h] = (high << 4) | low;
  }
  return h;
}


uint8_t Doppler_read(uint8_t data)
{
  static uint8_t df = 0;
  static uint8_t cnt = 0;
  static uint8_t rxbuff[20];
  
  uint16_t sin;
  volatile uint16_t sout;
  
  switch (df)
  {
  case 0:
    if (data == 0x7E) { rxbuff[cnt++] = data; df = 1; }
    break;
    
  case 1:
    if (data == 0x7E) { rxbuff[cnt++] = data; df = 2; }
    else { df = 0; cnt = 0; }
    break;
    
  case 2:
    rxbuff[cnt++] = data;
    if (cnt == 16)
    {
      // 테일 EF EF 검사 → 프레임 유효성
      if ((rxbuff[14] == 0xEF) && (rxbuff[15] == 0xEF))
      {
        // sin: 입력속도, sout: 출력속도(미사용)
        sin  = (rxbuff[6] << 8) | rxbuff[7];
        sout = (rxbuff[9] << 8) | rxbuff[10];
        
        LED_s = 1; // LED 점등 타이밍 유지
        
        // 최저 속도 조건(SPEED_MIN) 충족 시에만 "유효 샘플"로 표시
        if (sin >= (uint16_t)SPEED_MIN) {
          Speed       = (uint16_t)sin;   // 전역 Speed 갱신
          Speed_revf  = 1;               // 새 샘플 도착 알림
        }
        
        // 디버그 스위치: PB0가 눌려있으면 0이어도 반영
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
          if (sin != 0) {
            Speed      = (uint16_t)sin;
            Speed_revf = 1;
          }
        }
        
        // 상태 복구
        cnt = 0; df = 0;
      }
      else {
        // 잘못된 프레임 → 버퍼 리셋
        cnt = 0; df = 0;
      }
    }
    break;
  }
  
  return 0;
}

void SpeedLoop(void)
{
  static uint8_t sf = 0;
  static uint8_t  phase = 0;     // 0=대기, 1=숫자, 2=문구
  static uint16_t lock_speed = 0;
  static uint8_t  lock_kind  = 1;
  
  switch (sf)
  {
  case 0:
    if (T_100ms != 0) return;
    
    if (Speed_revf == 1)
    {
      Speed_revf = 0;
      sf = 2;
      T_100ms = update_s;
    }
    break;
    
  case 2:
    if (T_100ms != 0) return;
    
    if (Speed_revf == 1)
    {
      Speed_revf = 0;
      T_100ms = update_s;
      
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {
        Speed_Limit = 10;
      }
      
      if (cnt_on <= 0)
      {
        if (Speed >= SPEED_MIN)
        {
          // 1) 이번 샘플부터 먼저 누적
          tmp_speed   += Speed;
          speed_cnt_3 += 1;
          
          if (speed_cnt_3 >= 3)
          {
            int avg = tmp_speed / speed_cnt_3;   // 3회 평균 (정수)
            if (avg >= 10)
            {
              Rey_s  = 2;
              cnt_on = 1;
              
              int delta     = (int)Speed - (int)Speed_Limit;
              uint8_t kind  = 1; // 1 서행, 2 주의, 3 감속
              uint8_t color = COLOR_GREEN;
              if (delta >= 0)        { kind = 3; color = COLOR_RED;    }
              else if (delta >= -10) { kind = 2; color = COLOR_YELLOW; }
              
              lock_speed = (uint16_t)Speed;
              lock_kind  = kind;
              SpeedDisplay(color, lock_speed);
              T_blink = NUM_TICKS;   // 1.0s
              phase = 1;
              sf    = 10;
            }
            // 2) 한 사이클 끝났으니 리셋 (다음 3샘플을 새로 모음)
            tmp_speed   = 0;
            speed_cnt_3 = 0;
          }
        }
        else {
          // SPEED_MIN 미만이면 사이클 초기화
          tmp_speed   = 0;
          speed_cnt_3 = 0;
        }
      }
    }
    else
    {
      tmp_speed   = 0;
      speed_cnt_3 = 1;
      sf = 10;
    }
    break;
    
  case 10:
    // ?? 가드 수정: 둘 중 하나라도 살아있으면 대기
    if ((T_100ms != 0) || (T_blink != 0)) return;
    
    if (phase == 1)  // 숫자 → 문구 전환
    {
      // (기존) BusyDelayTicks(TICKS_400MS);  → (수정) 400ms 여유
      WaitMs(400);
      
      if      (lock_kind == 3) SendKorean_GamSok (COLOR_RED);
      else if (lock_kind == 2) SendKorean_JuYi   (COLOR_YELLOW);
      else                     SendKorean_SeoHang(COLOR_GREEN);
      
      T_blink = TXT_TICKS;   // 1.0s
      phase   = 2;
      return;
    }
    else if (phase == 2) // 문구 → OFF 전환
    {
      // (기존) Wait_100ms(5);   → (수정) 동일 의도면 500ms, 이전과 맞추려면 400ms
      WaitMs(400);
      
      SpeedDisplayOff();
      phase = 0;
      sf    = 0;
      return;
    }
    else
    {
      sf = 0;
      return;
    }
  }
}



void Set_Limit(void)
{
  uint8_t sw = 0;
  
  sw |= !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) << 0;
  sw |= !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) << 1;
  sw |= !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) << 2;
  sw |= !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) << 3;
  
  
  switch (sw)
  {
  case 0:       Speed_Limit = 100;        break;
  case 1:       Speed_Limit = 110;        break;
  case 2:       Speed_Limit = 120;        break;
  case 3:       Speed_Limit = 30;         break;
  case 4:       Speed_Limit = 40;         break;
  case 5:       Speed_Limit = 50;         break;
  case 6:       Speed_Limit = 60;         break;
  case 7:       Speed_Limit = 70;         break;
  case 8:       Speed_Limit = 80;         break;
  case 9:       Speed_Limit = 90;         break;
  
  default:      Speed_Limit = 60;         break;
  }
  
  
#if TEST_MODE == 1
  Speed_Limit = 15;
#endif
  
  
}


// ===== 정확한 밀리초 대기 (인터럽트/USART는 계속 동작) =====
static void WaitMs(uint32_t ms)
{
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < ms) {
    Usart_RxLoop();   // 대기 중에도 RX 파이프 비워서 프레임 누락 방지
  }
}

int main(void) 
{
  
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();
  MX_ADC_Init();
  
  
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim14);
  
  
  REY_OFF;
  
  
  Color = COLOR_GREEN;
  SpeedCnt = 0;
  
  Set_Limit();
  
  init(&q1);
  init(&q2);
  
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  
  M30_SetDefaultUart(&huart1);
  
  // ★ 전광판이 30초 후 ON → 여기서 정확히 35초 대기(콜드부트에서도 확실)
  WaitMs(35000);
  
  Init_s   = 5;
  update_s = 3;
  
  /* 아래 HAL_Delay(400) → 바쁜 대기로 교체 */
  WaitMs(400);
  SpeedDisplayOff();
  WaitMs(400);
  
  SendKorean_GamSok(1);
  WaitMs(400);
  SpeedDisplayOff();
  WaitMs(400);
  
  SendKorean_JuYi(2);
  WaitMs(400);
  SpeedDisplayOff();
  WaitMs(400);
  
  SendKorean_SeoHang(3);
  WaitMs(400);
  
  SpeedDisplayOff();
  
  while (1)
  {
    /* 1. 수신 처리 */
    Usart_RxLoop();
    
    /* 2. 초기 표시 상태머신 */
    switch (Init_f)
    {
    case 0:
      if (Init_s == 0) { Init_f = 1; Init_s = 1; }
      break;
    case 1:
      if (Init_s == 0) {
        Set_Limit();
        SpeedDisplay(COLOR_RED, Speed_Limit);
        Init_f = 2;
        Init_s = 2;
      }
      break;
    case 2:
      if (Init_s == 0) { SpeedDisplayOff(); Init_f = 3; }
      break;
    case 3:
      SpeedLoop();
      break;
    }
    
    /* 3. 원본 로직: cnt_on / Rey_s 제어 */
    if (cnt_on >= 1)
    {
      cnt_on++;
      Rey_s = 2;                  // 경광등 ON
    }
    
    if (cnt_on > 350000)           // 임계값 넘으면 초기화
    {
      cnt_on = 0;
      Rey_s = 0;                   // 경광등 OFF
    }
    
    /* 4. 1초 주기 처리 */
    if (T_f == 1)
    {
      T_f = 0;
      Set_Limit();
    }
    
    /* 5. 출력 제어 */
    if (Rey_s > 0) REY_ON; else REY_OFF;
    if (LED_s > 0) LED_ON; else LED_OFF;
  }
}



/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };
  
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
    | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief ADC Initialization Function
* @param None
* @retval None
*/
static void MX_ADC_Init(void)
{
  
  /* USER CODE BEGIN ADC_Init 0 */
  
  /* USER CODE END ADC_Init 0 */
  
  ADC_ChannelConfTypeDef sConfig = { 0 };
  
  /* USER CODE BEGIN ADC_Init 1 */
  
  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */
  
  /* USER CODE END ADC_Init 2 */
  
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  
  /* USER CODE END I2C1_Init 2 */
  
}

/**
* @brief TIM6 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM6_Init(void)
{
  
  /* USER CODE BEGIN TIM6_Init 0 */
  
  /* USER CODE END TIM6_Init 0 */
  
  /* USER CODE BEGIN TIM6_Init 1 */
  
  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 4799;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  
  /* USER CODE END TIM6_Init 2 */
  
}

/**
* @brief TIM14 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM14_Init(void)
{
  
  /* USER CODE BEGIN TIM14_Init 0 */
  
  /* USER CODE END TIM14_Init 0 */
  
  /* USER CODE BEGIN TIM14_Init 1 */
  
  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 4799;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */
  
  /* USER CODE END TIM14_Init 2 */
  
}

/**
* @brief USART1 Initialization Function
* @param None
* @retval None
*/
static void MX_USART1_UART_Init(void)
{
  
  /* USER CODE BEGIN USART1_Init 0 */
  
  /* USER CODE END USART1_Init 0 */
  
  /* USER CODE BEGIN USART1_Init 1 */
  
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  
  /* USER CODE END USART1_Init 2 */
  
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_6, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
  
  /*Configure GPIO pins : PA1 PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*Configure GPIO pins : PB0 PB12 PB13 PB14
  PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14
    | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pin : PF6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  
}

/* USER CODE BEGIN 4 */

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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */