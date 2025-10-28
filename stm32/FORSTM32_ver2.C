/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body (최종 통합본)
 ******************************************************************************
 * @attention
 *
 * ============================================================================
 * @refactor_final : Gemini (2025-10-28)
 * @description    : SW_1_(new).c (Linux) 로직을 STM32 Bare-metal로 포팅.
 *
 * @architecture   : '샘플(Sample) 기반' 스트리밍 아키텍처 채택.
 * - '블록(Block) 기반' (GPT 코드)은 메모리 과다 및 n_ta=0 오류로 폐기.
 * - ADC 샘플 1개 수신 시, 파이프라인 즉시 처리/누적.
 * - 'decim_rate' 개수 누적 시, 최종 결과(Stage 4~9) 1프레임 송신.
 *
 * @pipeline       : (1)Raw -> (2)LPF+Smooth -> (3)TimeAvg -> (4)R -> (5)Ravg
 * (6)y1 -> (7)y2 -> (8)y3 -> (9)yt -> (10)UART Frame
 *
 * @protocol_rx    : PC -> PCB (27개 설정값)
 * st|lpf|fs|frate|ma_r|ma_ch|mask|blk|y1_den[6]|y2[6]|y3[6]|yt[2]|end
 *
 * @protocol_tx    : PCB -> PC (메타 5 + 페이로드 24 = 29개 값)
 * st|sid|ts|fs(Input)|blk(Input)|mask|RAW8[8],RAVG4[4],Y2/3/T[12]|end
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

/* SW_1 DSP 파이프라인 파라미터 (PC -> PCB 수신용)
 * (부모 코드 및 GPT 코드의 변수명으로 통일)
 */
typedef struct {
  /* 스칼라 7개 */
  float     lpf_cutoff_hz;        // 1. LPF 컷오프 (현재 SOS 필터 고정, 문서용)
  float     sampling_rate;        // 2. HW 샘플레이트 (Fs)
  float     target_rate;          // 3. 출력 레이트 (Frate)
  int       movavg_r;             // 4. Stage 5 (Ravg) 이동평균 윈도우
  int       movavg_ch;            // 5. Stage 2 (Smooth) 이동평균 윈도우
  int       channel_mask;         // 6. 채널 마스크 (예: 0xFF)
  int       block_size;           // 7. 입력 블록 크기 (TX 메타데이터 전송용)

  /* R = (alpha*beta*gamma)*log_k(I_s/I_b) + b (부모 코드 고정값) */
  double    alpha, beta, gamma, k, b;
  int       r_abs;

  /* y-chain 계수 4묶음 (총 20개) */
  double    y1_den[6];      int y1_den_len;   // 8-13. y1 분모 (a5-a0)
  double    y1_num[2];      int y1_num_len;   // (고정값 1.0, 0.0)
  double    y2_coeffs[6];   int y2_coeffs_len;// 14-19. y2 계수 (b5-b0)
  double    y3_coeffs[6];   int y3_coeffs_len;// 20-25. y3 계수 (c5-c0)
  double    E, F;                           // 26-27. yt 계수 (E, F)

  /* 파생/내부 값 */
  int       decim_rate;           // Fs / Frate
} dsp_params_t;


/* DSP 런타임 상태 */
#define N_CH 8
#define N_QUAD 4
#define SOS_SECTIONS 2
#define MAX_MA_WIN 256  // 이동평균 최대 윈도우 (메모리 절약)

typedef struct {
  /* Stage 2: LPF 상태 (채널 8 x 섹션 2 x 상태 2) */
  double    lpf_state[N_CH][SOS_SECTIONS * 2];

  /* Stage 2: Smoothing (MA) 상태 (롤링 버퍼) */
  float     ma_ch_buf[N_CH][MAX_MA_WIN];
  int       ma_ch_pos[N_CH];
  double    ma_ch_sum[N_CH];

  /* Stage 3: Time Average (누적기) */
  double    avg_buf[N_CH];
  int       avg_count;

  /* Stage 5: Ravg (MA) 상태 (롤링 버퍼) */
  float     ma_r_buf[N_QUAD][MAX_MA_WIN];
  int       ma_r_pos[N_QUAD];
  double    ma_r_sum[N_QUAD];

  /* Stage 10: Output */
  uint32_t  frame_sid;          // 프레임 시퀀스 ID
} dsp_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RDV2_RX_BUFSZ    1024  // 수신 버퍼 (설정 프레임이 길어짐)
#define RDV2_TX_BUFSZ    2048  // 송신 버퍼 (데이터 프레임이 길어짐)

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
static dsp_params_t g_params; // DSP 설정
static dsp_state_t  g_state;  // DSP 런타임 상태

/* 통신 버퍼/상태 */
static uint8_t  g_rx_byte;
static char     g_rx_line[RDV2_RX_BUFSZ];
static uint16_t g_rx_len = 0;

static char     g_tx_line[RDV2_TX_BUFSZ];

/* 샘플/연산 결과 (PCB -> PC 전송용, 24개 float) */
static float    g_latest_results[24]; // [0:8]=RAW8, [8:12]=RAVG4, [12:24]=Y2,3,T
static uint8_t  g_send_flag = 0;      // 1이 되면 main loop에서 전송

/* SW_1 포팅: R 계산용 채널 인덱스 (고정) */
static const int sensor_idx[4]   = {0, 2, 4, 6};
static const int standard_idx[4] = {1, 3, 5, 7};

/* SW_1 포팅: LPF(SOS) 계수 (고정) */
static const double SOS_COEFFS[SOS_SECTIONS][6] = {
    {3.728052e-09, 7.456103e-09, 3.728052e-09, 1.0, -1.971149e+00, 9.713918e-01},
    {1.0,          2.0,          1.0,          1.0, -1.987805e+00, 9.880500e-01},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
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

/* 다항식 계산 (Horner's method, c[0] = highest order) */
static inline double polyval_f64(const double* c, int len, double x) {
    double r = 0.0;
    for (int i = 0; i < len; i++) r = r * x + c[i];
    return r;
}

/* CSV 문자열을 double 배열로 파싱 (GPT 코드에서 차용) */
static int parse_csv_vec(const char* s, double* out, int maxn, int* out_n)
{
  int n=0; char buf[256]; strncpy(buf,s,sizeof(buf)-1); buf[sizeof(buf)-1]='\0';
  char* t = strtok(buf, ",");
  while (t && n < maxn) { out[n++] = strtod(t, NULL); t = strtok(NULL, ","); }
  *out_n = n; 
  /* (중요) 남는 공간 0.0으로 채우기 */
  for (int i = n; i < maxn; i++) out[i] = 0.0;
  return n>0;
}

/* ==========================================================
 * SW_1 포팅: DSP 파이프라인 (샘플 단위)
 * ========================================================== */

/* Stage 2: LPF (SOS DF2T, 샘플 단위) */
static float sos_df2t_sample(float x, const double sos_section[6], double state[2]) {
    const double b0 = sos_section[0], b1 = sos_section[1], b2 = sos_section[2];
    const double a1 = sos_section[4], a2 = sos_section[5];
    double z1 = state[0], z2 = state[1];
    
    const double yi = b0 * x + z1;
    z1 = b1 * x - a1 * yi + z2;
    z2 = b2 * x - a2 * yi;
    
    state[0] = z1;
    state[1] = z2;
    return (float)yi;
}

/* LPF 래퍼 (모든 섹션 적용) */
static float apply_lpf_sample(int ch, float x) {
    float y = x;
    for (int s = 0; s < SOS_SECTIONS; s++) {
        y = sos_df2t_sample(y, SOS_COEFFS[s], g_state.lpf_state[ch] + (s * 2));
    }
    return y;
}

/* Stage 2 (Smooth) / Stage 5 (Ravg)용 롤링 이동평균 (샘플 단위)
 * (Bare-metal에 최적화된 방식)
 */
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

/* DSP 상태 전체 리셋 */
static void DSP_Reset_State(void) {
    // g_state의 모든 런타임 버퍼, 카운터, 합계를 0으로 초기화
    memset(&g_state, 0, sizeof(dsp_state_t));
}

/* ==========================================================
 * DSP 메인 처리 루프 (ADC 샘플 1개 입력 시 호출됨)
 * ========================================================== */
static void DSP_Process_Sample(void)
{
    /* Stage 1: Raw Acquisition (AD7606) */
    if (!ADC_Process_IsDataReady()) {
        return; // 새 데이터 없음
    }
    
    int16_t raw_s16[N_CH];
    ADC_Process_GetData(raw_s16);

    float lpf_out[N_CH];
    const double r_inv_log_b = 1.0 / log(g_params.k > 1.0 ? g_params.k : 10.0);
    const double r_scale = g_params.alpha * g_params.beta * g_params.gamma;

    /* Stage 2: LPF + Smoothing (per-channel) */
    for (int c = 0; c < N_CH; c++) {
        float x = (float)raw_s16[c];
        
        /* (1) LPF (SOS Filter) */
        x = apply_lpf_sample(c, x);
        
        /* (2) Smoothing (Channel MA) (롤링 평균) */
        x = apply_rolling_average(x, g_params.movavg_ch, 
                                  &g_state.ma_ch_pos[c], 
                                  &g_state.ma_ch_sum[c], 
                                  g_state.ma_ch_buf[c]);
        
        lpf_out[c] = x;
    }

    /* Stage 3: Time Average (Accumulate) */
    for (int c = 0; c < N_CH; c++) {
        g_state.avg_buf[c] += (double)lpf_out[c];
    }
    g_state.avg_count++;

    /* Time Average 완료 (decim_rate 만큼 샘플 누적됨) */
    if (g_state.avg_count < g_params.decim_rate) {
        return; // 아직 누적 중
    }

    /* --- 여기서부터 1개의 출력 샘플(n_ta=1) 생성 --- */
    
    /* Stage 3: Time Average (Finalize) */
    float ta_out[N_CH];
    double avg_div = (double)g_state.avg_count;
    for (int c = 0; c < N_CH; c++) {
        ta_out[c] = (float)(g_state.avg_buf[c] / avg_div);
    }
    // 누적기 리셋
    memset(g_state.avg_buf, 0, sizeof(g_state.avg_buf));
    g_state.avg_count = 0;

    /* (PCB->PC) Payload[0:8] = RAW8 (Time Avg 결과) */
    memcpy(&g_latest_results[0], ta_out, sizeof(float) * N_CH);

    float ravg_out[N_QUAD];

    /* Stage 4-9 (Quad 0-3) */
    for (int q = 0; q < N_QUAD; q++) {
        const int si = sensor_idx[q];
        const int bi = standard_idx[q];

        /* Stage 4: R (Log Ratio) */
        double top = (double)ta_out[si];
        double bot = (double)ta_out[bi];
        if (g_params.r_abs) { 
            if (top < 0) top = -top; 
            if (bot < 0) bot = -bot; 
        }
        if (top < 1e-12) top = 1e-12;
        if (bot < 1e-12) bot = 1e-12;
        
        const double ratio = top / bot;
        const double log_ratio = log(ratio) * r_inv_log_b; // log_k
        const float R = (float)(r_scale * log_ratio + g_params.b);

        /* Stage 5: Ravg = MovingAverage(R) (롤링 평균) */
        ravg_out[q] = apply_rolling_average(R, g_params.movavg_r, 
                                            &g_state.ma_r_pos[q],
                                            &g_state.ma_r_sum[q],
                                            g_state.ma_r_buf[q]);

        /* Stage 6: y1 = P(Ravg) / Q(Ravg) */
        const double r_t = (double)ravg_out[q];
        const double y1n = polyval_f64(g_params.y1_num, g_params.y1_num_len, r_t);
        const double y1d = polyval_f64(g_params.y1_den, g_params.y1_den_len, r_t);
        const double y1  = y1n / ((fabs(y1d) < 1e-12) ? 1e-12 : y1d);

        /* Stage 7: y2 = poly(y1) */
        const double y2  = polyval_f64(g_params.y2_coeffs, g_params.y2_coeffs_len, y1);
        
        /* Stage 8: y3 = poly(y2) */
        const double y3  = polyval_f64(g_params.y3_coeffs, g_params.y3_coeffs_len, y2);
        
        /* Stage 9: yt = E*y3 + F */
        const double yt  = g_params.E * y3 + g_params.F;

        /* (PCB->PC) Payload[12:24] = Y2, Y3, YT (채널별 3개) */
        g_latest_results[12 + q*3 + 0] = (float)y2;
        g_latest_results[12 + q*3 + 1] = (float)y3;
        g_latest_results[12 + q*3 + 2] = (float)yt;
    }
    
    /* (PCB->PC) Payload[8:12] = RAVG4 */
    memcpy(&g_latest_results[8], ravg_out, sizeof(float) * N_QUAD);

    /* Stage 10: Output (Flag) */
    g_send_flag = 1; // main loop에서 전송하도록 플래그 설정
}


/* ==========================================================
 * 수신 (PC -> PCB) (신규 프로토콜 파서)
 * ========================================================== */
static void DSP_Parse_Settings(char* line)
{
  if (strncmp(line, "st|", 3) != 0) return;
  if (strstr(line, "|end") == NULL) return;

  char* tok[32]; // 토큰 포인터 배열
  int ntok = 0;
  
  // strtok는 원본을 수정하므로 line (g_rx_line의 복사본)을 사용
  char *p = strtok(line, "|");
  while (p && ntok < 32) {
    tok[ntok++] = p;
    p = strtok(NULL, "|");
  }

  /* * 프로토콜: st| 7개 스칼라 | 4개 배열 | end
   * 필드 개수 = 1(st) + 7(스칼라) + 4(배열) + 1(end) = 13개
   */
  if (ntok != 13) return;
  if (strcmp(tok[0], "st") != 0) return;
  if (strcmp(tok[ntok - 1], "end") != 0) return;

  dsp_params_t np = g_params; // 복사본에 파싱

  /* 스칼라 7개 */
  np.lpf_cutoff_hz    = (float)strtof(tok[1],  NULL);
  np.sampling_rate    = (float)strtof(tok[2],  NULL);
  np.target_rate      = (float)strtof(tok[3],  NULL);
  np.movavg_r         = (int)strtol(tok[4], NULL, 10);
  np.movavg_ch        = (int)strtol(tok[5], NULL, 10);
  np.channel_mask     = (int)strtol(tok[6], NULL, 10);
  np.block_size       = (int)strtol(tok[7], NULL, 10);

  /* 계수(배열) 4묶음 (총 20개 값) */
  double yt_tmp[2]; int yt_n = 0;
  parse_csv_vec(tok[8],  np.y1_den,      6, &np.y1_den_len);
  parse_csv_vec(tok[9],  np.y2_coeffs,   6, &np.y2_coeffs_len);
  parse_csv_vec(tok[10], np.y3_coeffs,   6, &np.y3_coeffs_len);
  parse_csv_vec(tok[11], yt_tmp,         2, &yt_n);
  if (yt_n == 2) { np.E = yt_tmp[0]; np.F = yt_tmp[1]; }

  /* 파라미터 보정 및 파생값 계산 */
  if (np.sampling_rate < 1.0f) np.sampling_rate = 1.0f;
  if (np.target_rate < 0.1f) np.target_rate = 0.1f;
  
  np.decim_rate = (int)(np.sampling_rate / np.target_rate + 0.5f); // 반올림
  if (np.decim_rate < 1) np.decim_rate = 1;

  if (np.movavg_r < 1) np.movavg_r = 1;
  if (np.movavg_ch < 1) np.movavg_ch = 1;
  if (np.movavg_r > MAX_MA_WIN) np.movavg_r = MAX_MA_WIN;
  if (np.movavg_ch > MAX_MA_WIN) np.movavg_ch = MAX_MA_WIN;

  /* R 파라미터 (부모 코드 고정값) */
  np.alpha=1.0; np.beta=1.0; np.gamma=1.0; np.k=10.0; np.b=0.0; np.r_abs=1;
  np.y1_num[0]=1.0; np.y1_num[1]=0.0; np.y1_num_len=2;

  /* 새 파라미터 적용 */
  g_params = np;

  /* (중요) 파라미터 변경 시 DSP 상태 리셋 */
  DSP_Reset_State();
}

/* ==========================================================
 * 송신 (PCB -> PC) (신규 프로토콜 포맷터)
 * ========================================================== */
static void DSP_Send_Data_Frame(void)
{
  /* * 메타 5필드
   * 1. block_count    (g_state.frame_sid)
   * 2. timestamp_ms   (HAL_GetTick)
   * 3. sampling_rate  (g_params.sampling_rate, *입력 Fs*)
   * 4. block_size     (g_params.block_size, *입력 N*)
   * 5. channel_mask   (g_params.channel_mask)
   */
  const uint32_t block_count = g_state.frame_sid++;
  const uint64_t timestamp_ms = rdv2_millis();
  const float    sampling_rate_in = g_params.sampling_rate;
  const uint16_t block_size_in = (uint16_t)g_params.block_size;
  const uint16_t channel_mask = (uint16_t)g_params.channel_mask;

  /* 헤더(메타 5) 출력
   * (참고: 프로토콜 예시에 따라 Fs와 Block Size는 *입력* 기준값으로 전송)
   */
  int n = snprintf(g_tx_line, RDV2_TX_BUFSZ,
                   "st|%lu|%llu|%.1f|%u|%u|",
                   (unsigned long)block_count,
                   (unsigned long long)timestamp_ms,
                   sampling_rate_in,
                   (unsigned)block_size_in,
                   (unsigned)channel_mask);
  
  if (n < 0 || n >= RDV2_TX_BUFSZ) return;

  /* * 통합 페이로드 1필드 (총 24개 float, 콤마 구분)
   * [0:8]   = RAW8 (8)
   * [8:12]  = RAVG4 (4)
   * [12:24] = Y2,Y3,YT x 4 (12)
   */
  for (int i = 0; i < 23; i++) {
      n += snprintf(g_tx_line + n, RDV2_TX_BUFSZ - n,
                    "%.6f,", g_latest_results[i]);
      if (n < 0 || n >= RDV2_TX_BUFSZ) return;
  }
  
  /* 마지막 값 + |end\r\n */
  n += snprintf(g_tx_line + n, RDV2_TX_BUFSZ - n,
                "%.6f|end\r\n", g_latest_results[23]);
  if (n < 0 || n >= RDV2_TX_BUFSZ) return;

  /* 송신 */
  RDV2_DE_TX();
  HAL_UART_Transmit(&huart3, (uint8_t*)g_tx_line, (uint16_t)n, 100);
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

    /* \n 또는 |end 수신 시 라인 처리 */
    if (c=='\n' || (g_rx_len>=4 && strstr(g_rx_line, "|end")!=NULL)) {
      /* (임시) g_rx_line의 복사본을 만들어 파서에 전달 (strtok 때문) */
      char line_buf[RDV2_RX_BUFSZ];
      strncpy(line_buf, g_rx_line, g_rx_len);
      line_buf[g_rx_len] = '\0';
      
      /* 라인 끝의 \r, \n 제거 */
      for (int i = g_rx_len - 1; i >= 0; i--) {
          if (line_buf[i] == '\r' || line_buf[i] == '\n') {
              line_buf[i] = '\0';
          } else {
              break;
          }
      }

      DSP_Parse_Settings(line_buf); // 새 파서 호출
      
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
  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();    // USART3 보레이트 115200 (usart.c)
  MX_USB_OTG_HS_USB_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_FMC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

  /* ==== SW_1 DSP: 초기화 ==== */
  
  /* 파라미터 기본값 (PC에서 설정을 받기 전까지 사용) */
  memset(&g_params, 0, sizeof(dsp_params_t));
  g_params.sampling_rate    = 50000.0f; // AD 샘플링 (예)
  g_params.target_rate      = 10.0f;    // 출력 레이트 (예)
  g_params.decim_rate       = (int)(g_params.sampling_rate / g_params.target_rate + 0.5f);
  g_params.movavg_r         = 1;
  g_params.movavg_ch        = 1;
  g_params.channel_mask     = 0xFF;
  g_params.block_size       = 2048; // (TX 전송용 기본값)

  /* R 파라미터 (부모 코드 고정값) */
  g_params.alpha=1.0; g_params.beta=1.0; g_params.gamma=1.0; g_params.k=10.0; g_params.b=0.0; g_params.r_abs=1;
  g_params.y1_num[0]=1.0; g_params.y1_num[1]=0.0; g_params.y1_num_len=2;

  /* y-chain 기본값 (0으로 나누기 방지) */
  g_params.y1_den[0] = 1.0; g_params.y1_den_len = 6;
  g_params.y2_coeffs[0] = 1.0; g_params.y2_coeffs_len = 6;
  g_params.y3_coeffs[0] = 1.0; g_params.y3_coeffs_len = 6;
  g_params.E = 1.0; g_params.F = 0.0;
  
  /* DSP 상태 리셋 */
  DSP_Reset_State();
  
  /* g_latest_results 초기화 */
  memset(g_latest_results, 0, sizeof(g_latest_results));

  /* RS-485 수신 시작: USART3 바이트 인터럽트 */
  HAL_UART_Receive_IT(&huart3, &g_rx_byte, 1);

  /* AD7606 자동 샘플 시작 */
  ADC_Process_Init();

  /* DE 핀 RX로 */
  RDV2_DE_RX();

  /* Infinite loop */
  while (1)
  {
    /* * DSP 파이프라인 구동:
     * ADC에서 새 샘플을 가져와 Stage 1-3(누적)까지 처리.
     * Stage 3(Time Avg)이 완료되면 Stage 4-9를 마저 계산하고
     * g_send_flag를 1로 설정함.
     */
    DSP_Process_Sample();

    /* * 데이터 전송:
     * g_send_flag가 1이면(DSP_Process_Sample이 트리거),
     * 계산된 g_latest_results(24개)를 UART로 전송함.
     */
    if (g_send_flag) {
      g_send_flag = 0; // 플래그 리셋
      DSP_Send_Data_Frame();
    }
  }
}

/**
* @brief System Clock Configuration
* @retval None
*/


/* USER CODE BEGIN 4 */
/* (기존 코드와 동일) */
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