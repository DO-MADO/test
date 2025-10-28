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
* 개발 : 윤신요 , 일시 25.10.25
*
* 변경 이력 (2025-10-27):
*  - PCB→PC 송신 포맷을 PC 요구에 맞춰 "27값"으로 변경
*    (3블록×8=24 + nsamp + status + tempC = 27)
*  - Block1=RAW8, Block2=PROC8(보정+IIR), Block3=MA8(이동평균)
*  - 온도 계산 유지(ADS1115 자리), 송신 포함
*  - UART3 보레이트는 115200bps 사용 (주의: 실제 보레이트 설정은 usart.c)
******************************************************************************
*/
/* USER CODE END Header */
#include "main.h"
#include "adc_process.h"   // AD7606
#include "bsp_dac.h"       // DAC8831
#include "ads1115.h"       // ADS1115
#include "tim.h"           // TIM6, TIM7
#include "temp_controller.h"
#include <stdio.h>

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include "fmc.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float     lpf_cutoff_hz;        // 1
  float     sampling_rate;        // 2 (예: 50000.0f)
  float     adc_input_range;      // 3
  uint16_t  frame_decimation;     // 4 (전송 주기 제어; 1=트리거로만)
  uint16_t  avg_window;           // 5 (이동평균 창)
  uint16_t  flags;                // 6 (bit0: LPF ON, bit1: TEMP ON, bit2: MA ON)
  uint32_t  uart_baud;            // 7 (안내용; 실제 보레이트는 usart.c에서 설정)

  float     b_lpf[8];             // 채널별 IIR b 계수
  float     a_lpf[8];             // 채널별 IIR a 계수 (y = b*x + a*y_prev)
  float     gain[8];              // 채널별 gain
  float     offset[8];            // 채널별 offset

  /* 내부 상태 */
  float     z1[8];                // IIR 내부 상태
  float     ma_buf[8][128];       // 채널별 이동평균 버퍼
  uint16_t  ma_pos[8];
  float     ma_sum[8];

  uint32_t  sid;                  // 프레임 시퀀스
  uint32_t  deci_cnt;             // 전송 주기 카운터
} rdv2_params_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RDV2_CH          8
#define RDV2_RX_BUFSZ    512
#define RDV2_TX_BUFSZ    2048
#define RDV2_MAX_MA_WIN  128

/* RS-485 DE/RE 핀(있으면 사용, 없으면 매크로만 둠) */
#ifndef RDV2_DE_GPIO_Port
#define RDV2_DE_GPIO_Port GPIOB
#endif
#ifndef RDV2_DE_Pin
#define RDV2_DE_Pin       GPIO_PIN_1
#endif
#define RDV2_DE_TX()  HAL_GPIO_WritePin(RDV2_DE_GPIO_Port, RDV2_DE_Pin, GPIO_PIN_SET)
#define RDV2_DE_RX()  HAL_GPIO_WritePin(RDV2_DE_GPIO_Port, RDV2_DE_Pin, GPIO_PIN_RESET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static inline uint64_t rdv2_millis(void) { return HAL_GetTick(); }
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static rdv2_params_t g_rdv2;

/* 통신 버퍼/상태 */
static uint8_t  g_rx_byte;
static char     g_rx_line[RDV2_RX_BUFSZ];
static uint16_t g_rx_len = 0;

static char     g_tx_line[RDV2_TX_BUFSZ];

/* 샘플/연산 결과 */
static int16_t  g_raw8[RDV2_CH];    // AD7606 원시(int16)
static float    g_proc8[RDV2_CH];   // 최종값(여기선 MA 후)
static float    g_tempC = -999.0f;  // 온도(ADS1115)

/* 블록 버퍼 (송신용 3블록) */
static float    g_blk_raw8[RDV2_CH];   // RAW → float 변환값
static float    g_blk_proc8[RDV2_CH];  // 보정 + IIR
static float    g_blk_ma8[RDV2_CH];    // 이동평균 결과(최종)

/* 디버그/보유 */
int16_t adc_values[8];
float currentTemperature = -999.0f;
float targetTemperatureMonitor = -1.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void rdv2_send_data_frame_once(void);
static void RDV2_Comm_ConsumeLine(void);
static void RDV2_FetchRaw_Run(void);
static void RDV2_Compute_Run(void);
static void RDV2_ReadTemperatureC_Run(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void parse_vec8_fn(const char* csv, float *dst){
  char buf[256]; strncpy(buf, csv, sizeof(buf)-1); buf[sizeof(buf)-1]='\0';
  int i=0; char *t = strtok(buf, ",");
  while (t && i<8){ dst[i++] = (float)strtof(t,NULL); t = strtok(NULL,","); }
  while (i<8){ dst[i++]=0.0f; }
}

/* ===================== 수신 파서 ============================
 * 지원 프레임:
 *  1) "st|REQ|end"    → 최신 1프레임 전송(트리거)
 *  2) 설정 프레임:
 *     st|lpf|fs|range|decim|avg|flags|baud|b[]|a[]|gain[]|offset[]|end
 * ========================================================== */
static void RDV2_Comm_ConsumeLine(void)
{
  if (g_rx_len == 0) return;

  char buf[RDV2_RX_BUFSZ];
  uint16_t n = (g_rx_len < (RDV2_RX_BUFSZ - 1)) ? g_rx_len : (RDV2_RX_BUFSZ - 1);
  memcpy(buf, g_rx_line, n);
  buf[n] = '\0';
  for (uint16_t i = 0; i < n; i++) {
    if (buf[i] == '\r' || buf[i] == '\n') { buf[i] = '\0'; break; }
  }

  if (strncmp(buf, "st|", 3) != 0) return;
  if (strstr(buf, "|end") == NULL) return;

  if (strcmp(buf, "st|REQ|end") == 0) {
    rdv2_send_data_frame_once();
    return;
  }

  char *tok[32]; int ntok = 0;
  char *p = strtok(buf, "|");
  while (p && ntok < 32) { tok[ntok++] = p; p = strtok(NULL, "|"); }

  if (ntok < 12) return;
  if (strcmp(tok[0], "st") != 0) return;
  if (strcmp(tok[ntok - 1], "end") != 0) return;

  rdv2_params_t np = g_rdv2;

  /* 스칼라 7개 */
  np.lpf_cutoff_hz    = (float)strtod(tok[1],  NULL);
  np.sampling_rate    = (float)strtod(tok[2],  NULL);
  np.adc_input_range  = (float)strtod(tok[3],  NULL);
  np.frame_decimation = (uint16_t)strtoul(tok[4], NULL, 10);
  np.avg_window       = (uint16_t)strtoul(tok[5], NULL, 10);
  np.flags            = (uint16_t)strtoul(tok[6], NULL, 10);
  np.uart_baud        = (uint32_t)strtoul(tok[7], NULL, 10);

  /* 벡터 4묶음 (각 8개) */
  parse_vec8_fn(tok[8],  np.b_lpf);
  parse_vec8_fn(tok[9],  np.a_lpf);
  parse_vec8_fn(tok[10], np.gain);
  parse_vec8_fn(tok[11], np.offset);

  if (np.frame_decimation == 0) np.frame_decimation = 1;
  if (np.avg_window == 0)       np.avg_window = 1;
  if (np.avg_window > RDV2_MAX_MA_WIN) np.avg_window = RDV2_MAX_MA_WIN;

  /* 내부 상태 리셋 */
  for (int i = 0; i < RDV2_CH; i++) {
    np.z1[i] = 0.0f;
    np.ma_pos[i] = 0;
    np.ma_sum[i] = 0.0f;
    for (int k = 0; k < RDV2_MAX_MA_WIN; k++) np.ma_buf[i][k] = 0.0f;
  }

  g_rdv2 = np;

  /* 참고: UART 보레이트 즉시 반영은 usart.c에서 처리해야 안전 */
}

/* ===================== 주기 전송 태스크 ===================== */
void RDV2_CommTask(void)
{
  if (g_rdv2.frame_decimation <= 1) return; // 1이면 트리거로만
  if (++g_rdv2.deci_cnt >= g_rdv2.frame_decimation) {
    g_rdv2.deci_cnt = 0;
    rdv2_send_data_frame_once();
  }
}

/* ===================== 온도 계산 (ADS1115 자리) ============= */
static float rdv2_ntc_to_celsius(float v_div, float v_ref, float r_fixed)
{
  if (v_div <= 0.0f) v_div = 1e-6f;
  if (v_div >= v_ref) v_div = v_ref - 1e-6f;
  const float r_ntc = r_fixed * (v_div / (v_ref - v_div));

  const float B   = 3988.0f;
  const float R25 = 10000.0f;
  const float T0K = 273.15f + 25.0f;

  const float invT = (1.0f/T0K) + (1.0f/B) * logf(r_ntc / R25);
  const float TK   = 1.0f / invT;
  return TK - 273.15f;
}

static void RDV2_ReadTemperatureC_Run(void)
{
  /* TODO: 실제 프로젝트에 맞게 ADS1115에서 전압을 읽으세요.
     float v_div = ADS1115_ReadVoltage(0);
  */
  float v_div = 1.000f;          // placeholder
  const float v_ref   = 3.300f;
  const float r_fixed = 10000.0f;

  g_tempC = rdv2_ntc_to_celsius(v_div, v_ref, r_fixed);
}

/* ===================== 연산 파이프라인 ====================== */
static float rdv2_apply_ma(int ch, float x)
{
  uint16_t W = g_rdv2.avg_window;
  if (W <= 1) return x;
  if (W > RDV2_MAX_MA_WIN) W = RDV2_MAX_MA_WIN;

  uint16_t pos = g_rdv2.ma_pos[ch];
  float old = g_rdv2.ma_buf[ch][pos];
  g_rdv2.ma_buf[ch][pos] = x;
  g_rdv2.ma_pos[ch] = (uint16_t)((pos + 1) % W);

  g_rdv2.ma_sum[ch] += x - old;
  return g_rdv2.ma_sum[ch] / (float)W;
}

/* g_raw8[] → Block RAW8/PROC8/MA8 계산 */
static void RDV2_Compute_Run(void)
{
  /* Block #1: RAW8 (int16→float 변환) */
  for (int i=0;i<RDV2_CH;i++) {
    g_blk_raw8[i] = (float)g_raw8[i];
  }

  /* Block #2: 보정 + IIR */
  for (int i=0;i<RDV2_CH;i++){
    float x = (g_blk_raw8[i] - g_rdv2.offset[i]) * g_rdv2.gain[i];
    if (g_rdv2.flags & 0x0001){  // LPF ON
      float y = g_rdv2.b_lpf[i]*x + g_rdv2.a_lpf[i]*g_rdv2.z1[i];
      g_rdv2.z1[i] = y;
      x = y;
    }
    g_blk_proc8[i] = x;
  }

  /* Block #3: 이동평균(MA) */
  for (int i=0;i<RDV2_CH;i++){
    float x = g_blk_proc8[i];
    if (g_rdv2.flags & 0x0004){  // MA ON
      x = rdv2_apply_ma(i, x);
    }
    g_blk_ma8[i] = x;
    g_proc8[i]   = x; // 호환: 최종값
  }
}

/* ===================== RAW 추출 (AD7606) ==================== */
static void RDV2_FetchRaw_Run(void)
{
  if (ADC_Process_IsDataReady()){
    int16_t tmp[RDV2_CH];
    ADC_Process_GetData(tmp);
    for (int i=0;i<RDV2_CH;i++) g_raw8[i] = tmp[i];
  }
}

/* ===================== 송신 (27값 포맷) ===================== */
/*
 * PCB → PC 프레임 (총 27 값)
 *  헤더: st|sid|ts_ms|fs|nsamp|status|
 *  본문: RAW8 CSV \r\n
 *        PROC8 CSV \r\n
 *        MA8 CSV, nsamp, status, tempC |end\r\n
 */
static void rdv2_send_data_frame_once(void)
{
  /* 데이터 취득 & 연산 */
  RDV2_FetchRaw_Run();    // 50 kS/s 처리 루프 중 최신 샘플 사용
  RDV2_Compute_Run();     // RAW→PROC→MA
  if (g_rdv2.flags & 0x0002) {    // TEMP ON
    RDV2_ReadTemperatureC_Run();
  }

  /* 메타 */
  const uint32_t fs    = (uint32_t)g_rdv2.sampling_rate;  // 예: 50000
  const uint64_t ts    = rdv2_millis();
  const uint16_t nsamp = 1;   // 현재 프레임당 샘플 수(문자열 포맷상 보조 필드)
  const uint16_t status= 0;   // 상태 비트(예비)

  /* 헤더 출력 */
  int n = snprintf(g_tx_line, RDV2_TX_BUFSZ,
                   "st|%lu|%llu|%lu|%u|%u|",
                   (unsigned long)g_rdv2.sid++,
                   (unsigned long long)ts,
                   (unsigned long)fs,
                   nsamp, status);
  if (n < 0 || n >= RDV2_TX_BUFSZ) return;

  /* Block #1: RAW8 */
  n += snprintf(g_tx_line + n, RDV2_TX_BUFSZ - n,
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,\r\n",
                g_blk_raw8[0], g_blk_raw8[1], g_blk_raw8[2], g_blk_raw8[3],
                g_blk_raw8[4], g_blk_raw8[5], g_blk_raw8[6], g_blk_raw8[7]);
  if (n < 0 || n >= RDV2_TX_BUFSZ) return;

  /* Block #2: PROC8 */
  n += snprintf(g_tx_line + n, RDV2_TX_BUFSZ - n,
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,\r\n",
                g_blk_proc8[0], g_blk_proc8[1], g_blk_proc8[2], g_blk_proc8[3],
                g_blk_proc8[4], g_blk_proc8[5], g_blk_proc8[6], g_blk_proc8[7]);
  if (n < 0 || n >= RDV2_TX_BUFSZ) return;

  /* Block #3: MA8 + 보조3(nsamp,status,tempC) → 총 27 */
  n += snprintf(g_tx_line + n, RDV2_TX_BUFSZ - n,
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%.2f|end\r\n",
                g_blk_ma8[0], g_blk_ma8[1], g_blk_ma8[2], g_blk_ma8[3],
                g_blk_ma8[4], g_blk_ma8[5], g_blk_ma8[6], g_blk_ma8[7],
                (unsigned)nsamp, (unsigned)status, g_tempC);
  if (n < 0 || n >= RDV2_TX_BUFSZ) return;

  /* 송신 */
  RDV2_DE_TX();
  HAL_UART_Transmit(&huart3, (uint8_t*)g_tx_line, (uint16_t)strlen(g_tx_line), 100);
  RDV2_DE_RX();
}
/* USER CODE END 0 */

/* ==== UART Rx 콜백: 라인 조립 ('\n' 또는 '|end' 감지 시 처리) ==== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    uint8_t c = g_rx_byte;
    if (g_rx_len < (RDV2_RX_BUFSZ-1)) {
      g_rx_line[g_rx_len++] = (char)c;
      g_rx_line[g_rx_len]   = '\0';
    } else {
      g_rx_len = 0; g_rx_line[0]='\0'; // overflow → 리셋
    }

    if (c=='\n' || (g_rx_len>=4 && strstr(g_rx_line, "|end")!=NULL)) {
      RDV2_Comm_ConsumeLine();
      g_rx_len = 0; g_rx_line[0]='\0';
    }

    HAL_UART_Receive_IT(&huart3, &g_rx_byte, 1);
  }
}

/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();    // USART3 보레이트는 **usart.c**에서 115200으로 설정하세요.
  MX_USB_OTG_HS_USB_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_FMC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

  /* ==== RAWDATA v2: 초기화 ==== */
  g_rdv2 = (rdv2_params_t){0};
  g_rdv2.lpf_cutoff_hz   = 500.0f;
  g_rdv2.sampling_rate   = 50000.0f;      // AD 내부 샘플링(50 kS/s)
  g_rdv2.adc_input_range = 10.0f;
  g_rdv2.frame_decimation= 1;             // 1=트리거 요청으로만 전송
  g_rdv2.avg_window      = 1;             // MA OFF 기본
  g_rdv2.flags           = 0x0002;        // TEMP ON만 기본(필요 시 0x0007: LPF+TEMP+MA)
  g_rdv2.uart_baud       = 115200;        // 안내용(실설정은 usart.c)

  for (int i=0;i<RDV2_CH;i++){
    g_rdv2.b_lpf[i] = 1.0f;
    g_rdv2.a_lpf[i] = 0.0f;
    g_rdv2.gain[i]  = 1.0f;
    g_rdv2.offset[i]= 0.0f;
    g_rdv2.z1[i]    = 0.0f;
    g_rdv2.ma_pos[i]= 0;
    g_rdv2.ma_sum[i]= 0.0f;
    for (int k=0;k<RDV2_MAX_MA_WIN;k++) g_rdv2.ma_buf[i][k]=0.0f;

    g_blk_raw8[i]=0.0f; g_blk_proc8[i]=0.0f; g_blk_ma8[i]=0.0f;
  }
  g_rdv2.sid=0; g_rdv2.deci_cnt=0;

  /* RS-485 수신 시작: USART3 바이트 인터럽트 */
  HAL_UART_Receive_IT(&huart3, &g_rx_byte, 1);

  /* AD7606 자동 샘플 시작 (50kS/s) */
  ADC_Process_Init();

  /* (선택) DE 핀 RX로 */
  RDV2_DE_RX();

  /* (선택) 온도 컨트롤러 초기화 (비활성)
  TempController_Init();
  */

  /* Infinite loop */
  while (1)
  {
    /* 전송 주기 모드(원하면 frame_decimation>1로) */
    RDV2_CommTask();

    /* 필요 시, 다른 주기 작업 추가 */
  }
}

/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
    |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) { Error_Handler(); }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* MPU Configuration */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */