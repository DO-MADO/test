/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body (최종 통합본 - V3 검증 강화)
 ******************************************************************************
 * @attention
 *
 * ============================================================================
 * @refactor_final : Gemini (2025-10-28)
 * @description    : SW_1_(new).c (Linux) 로직을 STM32 Bare-metal로 포팅.
 * - 통신 프로토콜 검증 강화 (길이, mask base)
 * - 단위 통일 (PC 코드 기준: RX kS/s -> 내부 Hz, TX Hz -> kS/s)
 *
 * @architecture   : '샘플(Sample) 기반' 스트리밍 아키텍처 채택.
 * @pipeline       : (1)Raw -> (2)LPF+Smooth -> (3)TimeAvg -> (4)R -> (5)Ravg
 * (6)y1 -> (7)y2 -> (8)y3 -> (9)yt -> (10)UART Frame
 * @protocol_rx    : PC -> PCB (27개 설정값)
 * st|lpf|fs(kS/s)|frate(Hz)|ma_r|ma_ch|mask(int/hex)|blk|y1_den[6]|y2[6]|y3[6]|yt[2]|end
 * @protocol_tx    : PCB -> PC (메타 5 + 페이로드 24 = 29개 값)
 * st|sid|ts|fs(kS/s)|blk(Input)|mask|RAW8[8],RAVG4[4],Y2/3/T[12]|end
 * ============================================================================
 */
/* USER CODE END Header */
#include "main.h"
#include "adc_process.h"   // AD7606 (가정)
#include "bsp_dac.h"
#include "ads1115.h"
#include "tim.h"
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

/* SW_1 DSP 파이프라인 파라미터 */
typedef struct {
  /* 스칼라 7개 */
  float     lpf_cutoff_hz;        // 1. LPF 컷오프 (Hz)
  float     sampling_rate;        // 2. HW 샘플레이트 (내부 저장 단위: Hz)
  float     target_rate;          // 3. 출력 레이트 (Hz)
  int       movavg_r;             // 4. Stage 5 MA 윈도우
  int       movavg_ch;            // 5. Stage 2 MA 윈도우
  int       channel_mask;         // 6. 채널 마스크 (int)
  int       block_size;           // 7. 입력 블록 크기 (샘플 수)

  /* R 파라미터 */
  double    alpha, beta, gamma, k, b;
  int       r_abs;

  /* y-chain 계수 */
  double    y1_den[6];      int y1_den_len;
  double    y1_num[2];      int y1_num_len;
  double    y2_coeffs[6];   int y2_coeffs_len;
  double    y3_coeffs[6];   int y3_coeffs_len;
  double    E, F;

  /* 파생/내부 값 */
  int       decim_rate;           // Fs(Hz) / Frate(Hz)
} dsp_params_t;


/* DSP 런타임 상태 */
#define N_CH 8
#define N_QUAD 4
#define SOS_SECTIONS 2
#define MAX_MA_WIN 256

typedef struct {
  double    lpf_state[N_CH][SOS_SECTIONS * 2];
  float     ma_ch_buf[N_CH][MAX_MA_WIN];
  int       ma_ch_pos[N_CH];
  double    ma_ch_sum[N_CH];
  double    avg_buf[N_CH];
  int       avg_count;
  float     ma_r_buf[N_QUAD][MAX_MA_WIN];
  int       ma_r_pos[N_QUAD];
  double    ma_r_sum[N_QUAD];
  uint32_t  frame_sid;
} dsp_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RDV2_RX_BUFSZ    1024
#define RDV2_TX_BUFSZ    2048

/* RS-485 DE/RE 핀 */
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
static dsp_params_t g_params;
static dsp_state_t  g_state;
static uint8_t  g_rx_byte;
static char     g_rx_line[RDV2_RX_BUFSZ];
static uint16_t g_rx_len = 0;
static char     g_tx_line[RDV2_TX_BUFSZ];
static float    g_latest_results[24];
static uint8_t  g_send_flag = 0;
static const int sensor_idx[4]   = {0, 2, 4, 6};
static const int standard_idx[4] = {1, 3, 5, 7};
static const double SOS_COEFFS[SOS_SECTIONS][6] = {
    {3.728052e-09, 7.456103e-09, 3.728052e-09, 1.0, -1.971149e+00, 9.713918e-01},
    {1.0,          2.0,          1.0,          1.0, -1.987805e+00, 9.880500e-01},
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void); // Defined by CubeMX
static void MPU_Config(void);   // Defined by CubeMX
/* USER CODE BEGIN PFP */
static void DSP_Process_Sample(void);
static void DSP_Send_Data_Frame(void);
static void DSP_Parse_Settings(char* line);
static void DSP_Reset_State(void);
static inline double polyval_f64(const double* c, int len, double x);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==========================================================
 * SW_1 포팅: 헬퍼 함수
 * ========================================================== */

static inline double polyval_f64(const double* c, int len, double x) {
    double r = 0.0;
    for (int i = 0; i < len; i++) r = r * x + c[i];
    return r;
}

/* CSV 파싱 헬퍼 (개선: 길이 검증 위해 패딩 제거) */
static int parse_csv_vec(const char* s, double* out, int maxn, int* out_n)
{
  int n=0; char buf[256]; strncpy(buf,s,sizeof(buf)-1); buf[sizeof(buf)-1]='\0';
  char* t = strtok(buf, ",");
  while (t && n < maxn) { out[n++] = strtod(t, NULL); t = strtok(NULL, ","); }
  *out_n = n; // 실제 파싱된 개수 반환
  // 0.0 패딩 제거 - 호출부에서 길이 검증
  return n; // 실제 파싱된 개수 반환 (성공 여부 판단 기준 아님)
}

/* ==========================================================
 * SW_1 포팅: DSP 파이프라인 (샘플 단위)
 * ========================================================== */

static float sos_df2t_sample(float x, const double sos_section[6], double state[2]) {
    const double b0 = sos_section[0], b1 = sos_section[1], b2 = sos_section[2];
    const double a1 = sos_section[4], a2 = sos_section[5];
    double z1 = state[0], z2 = state[1];
    const double yi = b0 * x + z1;
    z1 = b1 * x - a1 * yi + z2;
    z2 = b2 * x - a2 * yi;
    state[0] = z1; state[1] = z2;
    return (float)yi;
}

static float apply_lpf_sample(int ch, float x) {
    float y = x;
    for (int s = 0; s < SOS_SECTIONS; s++) {
        y = sos_df2t_sample(y, SOS_COEFFS[s], g_state.lpf_state[ch] + (s * 2));
    }
    return y;
}

static float apply_rolling_average(float x, int N, int* pos_ptr, double* sum_ptr, float* buf) {
    if (N <= 1) return x;
    if (N > MAX_MA_WIN) N = MAX_MA_WIN;
    int pos = *pos_ptr;
    double old = (double)buf[pos];
    buf[pos] = x;
    *pos_ptr = (pos + 1) % N;
    double sum = *sum_ptr + (double)x - old;
    *sum_ptr = sum;
    return (float)(sum / (double)N);
}

static void DSP_Reset_State(void) {
    memset(&g_state, 0, sizeof(dsp_state_t));
}

/* DSP 메인 처리 루프 */
static void DSP_Process_Sample(void)
{
    if (!ADC_Process_IsDataReady()) return;
    int16_t raw_s16[N_CH];
    ADC_Process_GetData(raw_s16);

    float lpf_out[N_CH];
    const double r_inv_log_b = 1.0 / log(g_params.k > 1.0 ? g_params.k : 10.0);
    const double r_scale = g_params.alpha * g_params.beta * g_params.gamma;

    /* Stage 2: LPF + Smoothing */
    for (int c = 0; c < N_CH; c++) {
        float x = (float)raw_s16[c];
        x = apply_lpf_sample(c, x);
        x = apply_rolling_average(x, g_params.movavg_ch, &g_state.ma_ch_pos[c], &g_state.ma_ch_sum[c], g_state.ma_ch_buf[c]);
        lpf_out[c] = x;
    }

    /* Stage 3: Time Average (Accumulate) */
    for (int c = 0; c < N_CH; c++) g_state.avg_buf[c] += (double)lpf_out[c];
    g_state.avg_count++;

    if (g_state.avg_count < g_params.decim_rate) return; // 누적 중

    /* Stage 3: Time Average (Finalize) */
    float ta_out[N_CH];
    double avg_div = (double)g_state.avg_count;
    for (int c = 0; c < N_CH; c++) ta_out[c] = (float)(g_state.avg_buf[c] / avg_div);
    memset(g_state.avg_buf, 0, sizeof(g_state.avg_buf));
    g_state.avg_count = 0;
    memcpy(&g_latest_results[0], ta_out, sizeof(float) * N_CH); // RAW8 슬롯

    float ravg_out[N_QUAD];
    /* Stage 4-9 (Quad 0-3) */
    for (int q = 0; q < N_QUAD; q++) {
        const int si = sensor_idx[q], bi = standard_idx[q];
        /* Stage 4: R */
        double top = (double)ta_out[si], bot = (double)ta_out[bi];
        if (g_params.r_abs) { if (top < 0) top = -top; if (bot < 0) bot = -bot; }
        if (top < 1e-12) top = 1e-12; if (bot < 1e-12) bot = 1e-12;
        const float R = (float)(r_scale * (log(top / bot) * r_inv_log_b) + g_params.b);
        /* Stage 5: Ravg */
        ravg_out[q] = apply_rolling_average(R, g_params.movavg_r, &g_state.ma_r_pos[q], &g_state.ma_r_sum[q], g_state.ma_r_buf[q]);
        /* Stage 6: y1 */
        const double r_t = (double)ravg_out[q];
        const double y1n = polyval_f64(g_params.y1_num, g_params.y1_num_len, r_t);
        const double y1d = polyval_f64(g_params.y1_den, g_params.y1_den_len, r_t);
        const double y1  = y1n / ((fabs(y1d) < 1e-12) ? 1e-12 : y1d);
        /* Stage 7: y2 */
        const double y2  = polyval_f64(g_params.y2_coeffs, g_params.y2_coeffs_len, y1);
        /* Stage 8: y3 */
        const double y3  = polyval_f64(g_params.y3_coeffs, g_params.y3_coeffs_len, y2);
        /* Stage 9: yt */
        const double yt  = g_params.E * y3 + g_params.F;
        /* Payload 저장 */
        g_latest_results[12 + q*3 + 0] = (float)y2;
        g_latest_results[12 + q*3 + 1] = (float)y3;
        g_latest_results[12 + q*3 + 2] = (float)yt;
    }
    memcpy(&g_latest_results[8], ravg_out, sizeof(float) * N_QUAD); // RAVG4 슬롯
    g_send_flag = 1; // 전송 플래그
}

/* ==========================================================
 * 수신 (PC -> PCB) (프로토콜 검증 강화)
 * ========================================================== */
static void DSP_Parse_Settings(char* line)
{
  if (strncmp(line, "st|", 3) != 0 || !strstr(line, "|end")) return;

  char* tok[32]; int ntok = 0;
  char *p = strtok(line, "|");
  while (p && ntok < 32) { tok[ntok++] = p; p = strtok(NULL, "|"); }

  if (ntok != 13 || strcmp(tok[0], "st") != 0 || strcmp(tok[ntok - 1], "end") != 0) {
      // (선택) 오류 알림: HAL_UART_Transmit(&huart3, (uint8_t*)"NACK CFG FMT\r\n", 14, 100);
      return; // 필드 개수 또는 시작/끝 토큰 불일치
  }

  // 임시 파라미터 구조체 (검증 성공 시 g_params로 복사)
  dsp_params_t np = g_params;

  /* 스칼라 7개 파싱 */
  np.lpf_cutoff_hz    = (float)strtof(tok[1],  NULL);
  // [단위] kS/s 수신 -> Hz로 변환 저장
  np.sampling_rate    = (float)strtof(tok[2],  NULL) * 1000.0f;
  // [단위] Hz 수신 -> Hz로 저장
  np.target_rate      = (float)strtof(tok[3],  NULL);
  np.movavg_r         = (int)strtol(tok[4], NULL, 10);
  np.movavg_ch        = (int)strtol(tok[5], NULL, 10);
  // [수정] base 0 사용: 10진수/16진수 자동 감지
  np.channel_mask     = (int)strtol(tok[6], NULL, 0);
  np.block_size       = (int)strtol(tok[7], NULL, 10);

  /* 계수(배열) 4묶음 파싱 및 길이 검증 */
  double yt_tmp[2]; int y1d_n, y2c_n, y3c_n, yt_n;
  parse_csv_vec(tok[8],  np.y1_den,      6, &y1d_n);
  parse_csv_vec(tok[9],  np.y2_coeffs,   6, &y2c_n);
  parse_csv_vec(tok[10], np.y3_coeffs,   6, &y3c_n);
  parse_csv_vec(tok[11], yt_tmp,         2, &yt_n);

  // [검증] 각 배열의 길이가 정확히 맞는지 확인
  if (y1d_n != 6 || y2c_n != 6 || y3c_n != 6 || yt_n != 2) {
      // (선택) 오류 알림: HAL_UART_Transmit(&huart3, (uint8_t*)"NACK CFG LEN\r\n", 14, 100);
      return; // 길이 불일치 시 프레임 무시
  }
  // 길이가 맞으면 실제 적용
  np.y1_den_len = y1d_n;
  np.y2_coeffs_len = y2c_n;
  np.y3_coeffs_len = y3c_n;
  np.E = yt_tmp[0]; np.F = yt_tmp[1];

  /* 파라미터 보정 및 파생값 계산 */
  if (np.sampling_rate < 1.0f) np.sampling_rate = 1.0f; // Hz 기준
  if (np.target_rate < 0.1f) np.target_rate = 0.1f;     // Hz 기준
  np.decim_rate = (int)(np.sampling_rate / np.target_rate + 0.5f); // Hz/Hz
  if (np.decim_rate < 1) np.decim_rate = 1;
  if (np.movavg_r < 1) np.movavg_r = 1;
  if (np.movavg_ch < 1) np.movavg_ch = 1;
  if (np.movavg_r > MAX_MA_WIN) np.movavg_r = MAX_MA_WIN;
  if (np.movavg_ch > MAX_MA_WIN) np.movavg_ch = MAX_MA_WIN;
  // channel_mask 범위 검증 (0 ~ 255)
  if (np.channel_mask < 0 || np.channel_mask > 0xFF) np.channel_mask = 0xFF; // 잘못된 값은 기본값(전체)으로
  // block_size 검증
  if (np.block_size < 1) np.block_size = 1; // 최소 1

  /* R 파라미터 (고정값) */
  np.alpha=1.0; np.beta=1.0; np.gamma=1.0; np.k=10.0; np.b=0.0; np.r_abs=1;
  np.y1_num[0]=1.0; np.y1_num[1]=0.0; np.y1_num_len=2;

  /* 모든 검증 통과 시 실제 적용 */
  g_params = np;
  DSP_Reset_State(); // 상태 리셋
  // (선택) 성공 알림: HAL_UART_Transmit(&huart3, (uint8_t*)"ACK CFG OK\r\n", 12, 100);
}

/* ==========================================================
 * 송신 (PCB -> PC) (프로토콜 준수 확인)
 * ========================================================== */
static void DSP_Send_Data_Frame(void)
{
  const uint32_t block_count = g_state.frame_sid++;
  const uint64_t timestamp_ms = rdv2_millis();
  // [단위] 내부 Hz -> kS/s로 변환하여 전송
  const float    sampling_rate_out_ksps = g_params.sampling_rate / 1000.0f;
  const uint16_t block_size_in = (uint16_t)g_params.block_size;
  const uint16_t channel_mask = (uint16_t)g_params.channel_mask;

  /* 헤더(메타 5) 출력 */
  int n = snprintf(g_tx_line, RDV2_TX_BUFSZ,
                   "st|%lu|%llu|%.3f|%u|%u|", // sampling_rate 소수점 3자리까지 (kS/s)
                   (unsigned long)block_count,
                   (unsigned long long)timestamp_ms,
                   sampling_rate_out_ksps, // kS/s 단위 전송
                   (unsigned)block_size_in,
                   (unsigned)channel_mask);
  if (n < 0 || n >= RDV2_TX_BUFSZ) return;

  /* 통합 페이로드 1필드 (24개 float) */
  for (int i = 0; i < 23; i++) {
      n += snprintf(g_tx_line + n, RDV2_TX_BUFSZ - n, "%.6f,", g_latest_results[i]);
      if (n < 0 || n >= RDV2_TX_BUFSZ) return;
  }
  n += snprintf(g_tx_line + n, RDV2_TX_BUFSZ - n, "%.6f|end\r\n", g_latest_results[23]);
  if (n < 0 || n >= RDV2_TX_BUFSZ) return;

  /* 송신 */
  RDV2_DE_TX();
  HAL_UART_Transmit(&huart3, (uint8_t*)g_tx_line, (uint16_t)n, 100);
  RDV2_DE_RX();
}
/* USER CODE END 0 */

/* ==== UART Rx 콜백 ==== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    uint8_t c = g_rx_byte;
    if (g_rx_len < (RDV2_RX_BUFSZ-1)) {
      g_rx_line[g_rx_len++] = (char)c;
      g_rx_line[g_rx_len]   = '\0';
    } else {
      g_rx_len = 0; g_rx_line[0]='\0'; // overflow
    }

    if (c=='\n' || (g_rx_len>=4 && strstr(g_rx_line, "|end")!=NULL)) {
      char line_buf[RDV2_RX_BUFSZ];
      strncpy(line_buf, g_rx_line, g_rx_len); line_buf[g_rx_len] = '\0';
      for (int i = g_rx_len - 1; i >= 0; i--) { // \r, \n 제거
          if (line_buf[i] == '\r' || line_buf[i] == '\n') line_buf[i] = '\0';
          else break;
      }
      DSP_Parse_Settings(line_buf);
      g_rx_len = 0; g_rx_line[0]='\0'; // 버퍼 리셋
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
  MPU_Config();
  HAL_Init();
  SystemClock_Config(); // From CubeMX

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init(); // Baudrate check in usart.c
  MX_USB_OTG_HS_USB_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_FMC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

  /* ==== SW_1 DSP: 초기화 ==== */
  memset(&g_params, 0, sizeof(dsp_params_t));
  // [단위] 내부 Hz 기준으로 초기화
  g_params.sampling_rate    = 50000.0f; // Hz
  g_params.target_rate      = 10.0f;    // Hz
  g_params.decim_rate       = (int)(g_params.sampling_rate / g_params.target_rate + 0.5f);
  g_params.movavg_r         = 1;
  g_params.movavg_ch        = 1;
  g_params.channel_mask     = 0xFF;
  g_params.block_size       = 2048; // 샘플 수
  g_params.alpha=1.0; g_params.beta=1.0; g_params.gamma=1.0; g_params.k=10.0; g_params.b=0.0; g_params.r_abs=1;
  g_params.y1_num[0]=1.0; g_params.y1_num[1]=0.0; g_params.y1_num_len=2;
  g_params.y1_den[0] = 1.0; g_params.y1_den_len = 6;
  g_params.y2_coeffs[0] = 1.0; g_params.y2_coeffs_len = 6;
  g_params.y3_coeffs[0] = 1.0; g_params.y3_coeffs_len = 6;
  g_params.E = 1.0; g_params.F = 0.0;

  DSP_Reset_State();
  memset(g_latest_results, 0, sizeof(g_latest_results));

  HAL_UART_Receive_IT(&huart3, &g_rx_byte, 1); // UART 수신 시작
  ADC_Process_Init(); // AD7606 시작
  RDV2_DE_RX();       // RS485 수신 모드

  /* Infinite loop */
  while (1) {
    DSP_Process_Sample(); // ADC 샘플 처리
    if (g_send_flag) {
      g_send_flag = 0;
      DSP_Send_Data_Frame(); // 결과 전송
    }
  }
}




/* ============================================================================
 * [CubeMX 자동 생성 코드] 클럭(Clock), MPU, 에러 핸들러 등
 * ============================================================================
 * 주의: 아래 함수들 (SystemClock_Config, MPU_Config, Error_Handler 등)은
 * STM32CubeMX가 프로젝트(.ioc) 설정을 기반으로 자동 생성한 코드입니다.
 * 하드웨어 담당자는 이 설정이 실제 보드 구성 (크리스탈, 전원 등)과
 * 일치하는지 반드시 확인해야 합니다.
 * 특히 SystemClock_Config() 함수는 MCU 및 주변 장치의 동작 속도를
 * 결정하는 핵심 부분이므로, CubeMX의 'Clock Configuration' 탭 설정과
 * 동일한지 검토가 필요합니다.
 * ============================================================================ */

/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  
  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  
  /* Disables the MPU */
  HAL_MPU_Disable();
  
  /** Initializes and configures the Region and the memory to be protected
  */
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
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  
}

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
