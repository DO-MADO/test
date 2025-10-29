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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "adc_process.h"   // AD7606 (가정)
#include "bsp_dac.h"
#include "ads1115.h"
#include "ads1115_process.h"
#include "tim.h"
//#include "temp_controller.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* ============================================================================
 * DSP 파이프라인 설정 파라미터 구조체 (`g_params`)
 * ============================================================================
 * PC로부터 수신하는 '설정 프레임' (PC -> PCB)의 내용을 저장하고,
 * DSP 파이프라인의 각 단계 연산 방식을 결정하는 변수 모음입니다.
 *
 * [통신 프로토콜 (PC -> PCB)]
 * - 형식: st|{스칼라 7개}|{배열 4개}|end
 * - 구분자: 필드 '|', 배열 내부 ','
 * - 전체 값 개수: 27개 (스칼라 7 + 계수 20)
 * - 필드 순서:
 * 1. lpf_cutoff_hz (float, Hz)
 * 2. sampling_rate (float, kS/s) -> 내부 Hz 변환 저장
 * 3. target_rate (float, Hz)
 * 4. movavg_r (int, 샘플 수)
 * 5. movavg_ch (int, 샘플 수)
 * 6. channel_mask (int/hex, 0~255)
 * 7. block_size (int, 샘플 수)
 * 8. coeffs_y1 (y1_den) (double[6], a5~a0)
 * 9. coeffs_y2 (y2_coeffs) (double[6], b5~b0)
 * 10. coeffs_y3 (y3_coeffs) (double[6], c5~c0)
 * 11. coeffs_yt (E, F) (double[2])
 *
 * - 파싱 함수: DSP_Parse_Settings()
 * - 수신된 프레임의 형식(토큰 개수) 및 각 배열의 길이(6, 6, 6, 2) 검증.
 * - channel_mask는 10진수/16진수 자동 인식.
 * - 검증 실패 시 설정 적용 안 함.
 * - 검증 성공 시 g_params 업데이트 및 DSP_Reset_State() 호출.
 *
 *
 * 예시 코드
 * st|2500|1000|1024|128|64|255|2048|
   1.0,-1.2,0.5,0.0,0.0,0.0|
   1.0,2.0,3.0,0.0,0.0,0.0|
   0.05,0.10,0.15,0.0,0.0,0.0|
   0.8,0.2|end
 * ============================================================================
 */


typedef struct {
  /* 스칼라 7개 */
  /* --- 스칼라 7개 (PC -> PCB 수신 필드 1~7) --- */
  float     lpf_cutoff_hz;        // 1. LPF 컷오프 주파수 (Hz). (현재 코드 LPF 고정으로 미사용, 정보용)
  float     sampling_rate;        // 2. ADC 하드웨어 샘플링 속도 (수신: kS/s, 내부 저장/사용: Hz).
  float     target_rate;          // 3. 최종 출력 데이터 속도 (수신/저장/사용: Hz).
  int       movavg_r;             // 4. Stage 5 (Ravg) 이동평균 윈도우 크기 (샘플 수).
  int       movavg_ch;            // 5. Stage 2 (Smoothing) 이동평균 윈도우 크기 (샘플 수).
  int       channel_mask;         // 6. 채널 마스크 (0~255). (수신: int/hex, 내부 저장: int). PCB->PC 송신 시 사용.
  int       block_size;           // 7. ADC 처리 블록 크기 (샘플 수). (샘플 기반 코드에서는 미사용, PCB->PC 송신 시 사용)
  float     target_temp_c;        // 8. **신규 : 목표 온도온도(°C). PC가 안 보내면 디폴트 유지



  /* --- R 파라미터 (Stage 4) --- */
  // 이 값들은 현재 코드에서 PC로 설정받지 않고 내부 고정값으로 사용됩니다.
  double    alpha, beta, gamma; // R 비례 상수 (현재 1.0 고정).
  double    k;                  // R 로그 밑 (현재 10.0 고정).
  double    b;                  // R 오프셋 (현재 0.0 고정).
  int       r_abs;              // R 계산 시 절대값 사용 여부 (현재 1 고정).


  /* --- y-chain 계수 (PC -> PCB 수신 필드 8~11) --- */
  // Stage 6 ~ 9 연산에 사용되는 다항식/선형 변환 계수. PC에서 설정 가능.
  double    y1_den[6];          // 8. Stage 6: y1 분모 다항식 계수 (a5 ~ a0).
  int       y1_den_len;         // y1_den 실제 길이 (파싱 후 항상 6으로 설정됨).
  double    y1_num[2];          // Stage 6: y1 분자 다항식 계수 (1*r + 0). (내부 고정값).
  int       y1_num_len;         // y1_num 실제 길이 (항상 2).
  double    y2_coeffs[6];       // 9. Stage 7: y2 다항식 계수 (b5 ~ b0).
  int       y2_coeffs_len;      // y2_coeffs 실제 길이 (파싱 후 항상 6으로 설정됨).
  double    y3_coeffs[6];       // 10. Stage 8: y3 다항식 계수 (c5 ~ c0).
  int       y3_coeffs_len;      // y3_coeffs 실제 길이 (파싱 후 항상 6으로 설정됨).
  double    E, F;               // 11. Stage 9: yt 선형 변환 계수 (yt = E*y3 + F).



  /* --- 파생/내부 값 --- */
  // PC 설정값을 기반으로 계산되어 내부 로직에서 사용되는 값.
  int       decim_rate;         // Stage 3: 시간 평균 비율 = sampling_rate(Hz) / target_rate(Hz).
} dsp_params_t;


/* DSP 런타임 상태 */
#define N_CH 8
#define N_QUAD 4
#define SOS_SECTIONS 2
#define MAX_MA_WIN 256


// AD7606은 ±10V 입력 범위를 갖는 16비트 양극성 ADC이므로
// int16 카운트를 실제 전압[V]로 환산하기 위한 스케일 팩터를 정의한다.
// (32768 = 2^15, 양/음 대칭 풀스케일 카운트)
#define AD7606_INPUT_RANGE_V   (10.0f)
#define AD7606_COUNTS_FULLSCALE (32768.0f)
#define AD7606_COUNTS_TO_V      (AD7606_INPUT_RANGE_V / AD7606_COUNTS_FULLSCALE)


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

// 1. DAC 출력 설정 (외부 온도 컨트롤러의 목표값 설정 사양)
// 가정: 외부 컨트롤러가 0V~2.5V 입력을 받아 0°C~100°C로 설정됨.
#define TC_DAC_V_MIN (0.0f)
// TC_DAC_V_MAX는 bsp_dac.c의 BSP_DAC_VREF(기본 2.5f)와 일치해야 합니다.
#define TC_DAC_V_MAX (2.5f)
#define TC_TEMP_SET_MIN (0.0f)
#define TC_TEMP_SET_MAX (100.0f)

// 선형 변환을 위한 기울기 및 오프셋 계산 (V = m*T + b)
#define TC_T2V_SLOPE ((TC_DAC_V_MAX - TC_DAC_V_MIN) / (TC_TEMP_SET_MAX - TC_TEMP_SET_MIN))
#define TC_T2V_OFFSET (TC_DAC_V_MIN - (TC_TEMP_SET_MIN * TC_T2V_SLOPE))

// 2. NTC 온도 센서 설정 (기존 코드의 하드코딩된 값을 Define으로 이동)
#define NTC_V_REF (3.300f)      // NTC 회로의 기준 전압 (VCC)
#define NTC_R_FIXED (10000.0f)  // 고정 저항 값 (Ohm)
#define NTC_B_PARAM (3988.0f)   // NTC 서미스터 사양 (예: B57703M703)
#define NTC_R25 (10000.0f)      // 25°C에서의 NTC 저항 값
#define KELVIN_OFFSET (273.15f)
#define T0K (KELVIN_OFFSET + 25.0f)
/* ============================================================================ */

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
static float    g_tempC = -999.0f;     // 최신 실제 온도(°C)
static const int sensor_idx[4]   = {0, 2, 4, 6};
static const int standard_idx[4] = {1, 3, 5, 7};
static const double SOS_COEFFS[SOS_SECTIONS][6] = {
    {9.661397e-11, 1.932279e-10, 9.661397e-11, 1.000000e+00, -1.988418e+00, 9.884573e-01},
    {1.000000e+00, 2.000000e+00, 1.000000e+00, 1.000000e+00, -1.995163e+00, 9.952026e-01},
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

// [신규] 온도 제어 함수 프로토타입
static HAL_StatusTypeDef TC_Set_Target_Temperature(float temp_c);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==========================================================
 * [신규] 온도 제어 로직 (DAC 출력 설정)
 * ========================================================== */

/**
 * @brief 목표 온도를 설정하고 DAC 출력을 업데이트합니다. (내부적으로 °C -> V 변환 포함)
 */
static HAL_StatusTypeDef TC_Set_Target_Temperature(float temp_c)
{
    // 1. 입력 온도 범위 제한 (Clamping)
    if (temp_c < TC_TEMP_SET_MIN) temp_c = TC_TEMP_SET_MIN;
    if (temp_c > TC_TEMP_SET_MAX) temp_c = TC_TEMP_SET_MAX;

    // 2. 온도를 전압으로 변환 (선형 변환)
    float voltage = (temp_c * TC_T2V_SLOPE) + TC_T2V_OFFSET;

    // 3. 출력 전압 범위 최종 확인 (안전 코드)
    if (voltage < TC_DAC_V_MIN) voltage = TC_DAC_V_MIN;
    if (voltage > TC_DAC_V_MAX) voltage = TC_DAC_V_MAX;

    // 4. DAC 출력 설정 (BSP_DAC 라이브러리 사용)
    return BSP_DAC_SetVoltage(voltage);
}


/* ==========================================================
 * [기존 수정] 온도 관련 (NTC 계산 및 ADS1115 읽기)
 * ========================================================== */

/* ---- 온도(°C) 계산: NTC 분압 → °C (B-파라미터 근사) ---- */
// 기존 함수를 수정하여 파라미터 대신 Define 상수를 사용하도록 개선
static float ntc_voltage_to_celsius(float v_div)
{
  if (v_div <= 0.0f) v_div = 1e-6f;
  if (v_div >= NTC_V_REF) v_div = NTC_V_REF - 1e-6f;
  const float r_ntc = NTC_R_FIXED * (v_div / (NTC_V_REF - v_div));

  // Define된 상수 사용
  const float invT = (1.0f/T0K) + (1.0f/NTC_B_PARAM) * logf(r_ntc / NTC_R25);
  return (1.0f/invT) - KELVIN_OFFSET;
}


/* 현재 온도 갱신(예: ADS1115 0번 채널에서 분압 읽기) */
static void Update_Current_Temperature(void)
{
    float v_div = 0.0f; // NTC 전압 (채널 0)
    HAL_StatusTypeDef status0;

    // 1. "ads1115_process"가 새 데이터를 준비했는지 확인합니다.
    if (ADS1115_Process_IsDataReady())
    {
        // 2. "ads1115_process"로부터 값을 가져옵니다.
        ADS1115_Process_GetData(&v_div, NULL, &status0, NULL);

        if (status0 == HAL_OK)
        {
            // 3. NTC 변환 로직 수행 및 최종 온도 값을 전역 변수 g_tempC에 갱신
            g_tempC = ntc_voltage_to_celsius(v_div);
        }
        else
        {
            // ADC 읽기 실패 시
            g_tempC = -999.0f; // 에러 값
        }
    }
    // (IsDataReady()가 0이면 g_tempC는 이전 값을 유지)
}



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
    /* ========================================================== */
    /* Stage 1: Raw Acquisition (8채널, ADC 샘플 1개)              */
    /* ========================================================== */

    // ADC_Process_IsDataReady(): ADC 변환 완료 여부 확인 (adc_process.c/.h 필요)
    if (!ADC_Process_IsDataReady()) return;

    // ADC_Process_GetData(): ADC 결과(int16) 가져오기 (adc_process.c/.h 필요)
    int16_t raw_s16[N_CH];
    ADC_Process_GetData(raw_s16);

    float lpf_out[N_CH];  // Stage 2 결과를 담을 임시 버퍼

    // R 계산에 필요한 상수 로딩 (가독성 및 최적화)
    const double r_inv_log_b = 1.0 / log(g_params.k > 1.0 ? g_params.k : 10.0);
    const double r_scale = g_params.alpha * g_params.beta * g_params.gamma;

    /* ========================================================== */
    /* Stage 2: Noise Filter (LPF + Smoothing MA) (8채널)        */
    /* ========================================================== */


    for (int c = 0; c < N_CH; c++) {
      // (1) Raw S16 -> 실제 전압[V] 변환 (±10V 풀스케일 기준)
        float x = (float)raw_s16[c] * AD7606_COUNTS_TO_V;

        // (2) Low Pass Filter (고정된 SOS 계수 사용)
        // apply_lpf_sample(): sos_df2t_sample()을 내부적으로 호출하여 필터링
        x = apply_lpf_sample(c, x);

        // (3) Smoothing (Channel Moving Average) (롤링 평균 방식)
        // apply_rolling_average(): g_params.movavg_ch 윈도우 크기만큼 이동평균
        x = apply_rolling_average(x, g_params.movavg_ch, &g_state.ma_ch_pos[c], &g_state.ma_ch_sum[c], g_state.ma_ch_buf[c]);
        lpf_out[c] = x;  // Stage 2 결과 저장
    }

    /* ========================================================== */
    /* Stage 3: Time Average (시간 평균) (8채널)                   */
    /* ========================================================== */

    // (1) Accumulate (샘플 누적)
    for (int c = 0; c < N_CH; c++) g_state.avg_buf[c] += (double)lpf_out[c];
    g_state.avg_count++;

    // (2) Check & Finalize (decim_rate 개수만큼 누적되었는지 확인)
    // g_params.decim_rate = Fs(Hz) / Frate(Hz)
    if (g_state.avg_count < g_params.decim_rate) return; // 아직 누적 중이면 함수 종료

    // (3) Finalize (평균 계산)
    float ta_out[N_CH];   // Stage 3 최종 결과를 담을 버퍼
    double avg_div = (double)g_state.avg_count;
    for (int c = 0; c < N_CH; c++) ta_out[c] = (float)(g_state.avg_buf[c] / avg_div);

    // (4) Reset Accumulator (다음 평균을 위해 누적 변수 초기화)
    memset(g_state.avg_buf, 0, sizeof(g_state.avg_buf));
    g_state.avg_count = 0;

    // (5) 결과 저장 (PCB->PC 전송용 버퍼의 RAW8 영역에 저장)
    memcpy(&g_latest_results[0], ta_out, sizeof(float) * N_CH); // RAW8 슬롯

    float ravg_out[N_QUAD];  // Stage 5 결과를 담을 임시 버퍼


    /* ========================================================== */
    /* Stage 4 ~ 9: R 계산 및 y-chain 처리 (4채널 Quad)          */
    /* ========================================================== */


    for (int q = 0; q < N_QUAD; q++) {  // 4개의 Quad (0/1, 2/3, 4/5, 6/7 채널 쌍) 반복
        const int si = sensor_idx[q],  // Sensor 채널 인덱스
        bi = standard_idx[q];          // Standard 채널 인덱스


        /* Stage 4: R (Log Ratio) 계산 */
        double top = (double)ta_out[si], // 분자 (Sensor)
         bot = (double)ta_out[bi];       // 분모 (Standard)

         // 절대값 처리 (g_params.r_abs 설정에 따라)
        if (g_params.r_abs) { if (top < 0) top = -top; if (bot < 0) bot = -bot; }

        // 0 또는 음수 방지 (log 계산 오류 방지)
        if (top < 1e-12) top = 1e-12;
        if (bot < 1e-12) bot = 1e-12;

        // R = a * log_k(top/bot) + b
        const float R = (float)(r_scale * (log(top / bot) * r_inv_log_b) + g_params.b);


        /* Stage 5: Ravg (Moving Average of R) (롤링 평균 방식) */
        // apply_rolling_average(): g_params.movavg_r 윈도우 크기만큼 이동평균
        ravg_out[q] = apply_rolling_average(R, g_params.movavg_r, &g_state.ma_r_pos[q], &g_state.ma_r_sum[q], g_state.ma_r_buf[q]);

        /* Stage 6: y1 = P(Ravg) / Q(Ravg) (분수 다항식) */
        const double r_t = (double)ravg_out[q];  // Stage 5 결과 사용

        // polyval_f64(): Horner 방식으로 다항식 계산
        const double y1n = polyval_f64(g_params.y1_num, g_params.y1_num_len, r_t); // 분자 (고정: 1*r_t + 0)
        const double y1d = polyval_f64(g_params.y1_den, g_params.y1_den_len, r_t); // 분모 (PC 설정값)

        // 0으로 나누기 방지
        const double y1  = y1n / ((fabs(y1d) < 1e-12) ? 1e-12 : y1d);


        /* Stage 7: y2 = poly(y1) (다항식) */
        const double y2  = polyval_f64(g_params.y2_coeffs, g_params.y2_coeffs_len, y1);

        /* Stage 8: y3 = poly(y2) (다항식) */
        const double y3  = polyval_f64(g_params.y3_coeffs, g_params.y3_coeffs_len, y2);

        /* Stage 9: yt = E*y3 + F (선형 변환) */
        const double yt  = g_params.E * y3 + g_params.F;


        /* 결과 저장 (PCB->PC 전송용 버퍼의 y2, y3, yt 영역에 저장) */
        // 채널 우선(Channel-major) 인덱싱: y2_0, y3_0, yt_0, y2_1, y3_1, yt_1, ...
        g_latest_results[12 + q*3 + 0] = (float)y2;
        g_latest_results[12 + q*3 + 1] = (float)y3;
        g_latest_results[12 + q*3 + 2] = (float)yt;

    }
    // Stage 5 결과 저장 (PCB->PC 전송용 버퍼의 RAVG4 영역에 저장)
    memcpy(&g_latest_results[8], ravg_out, sizeof(float) * N_QUAD); // RAVG4 슬롯

    /* ========================================================== */
    /* Stage 10: Data Output (전송 플래그 설정)                   */
    /* ========================================================== */

    g_send_flag = 1; // 전송 플래그
                     // main() 루프에서 DSP_Send_Data_Frame()을 호출하도록 플래그 설정
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

  // 허용 토큰 수:
  //  - 레거시(스칼라7 + 배열4) = 13
  //  - 신규(스칼라8 + 배열4)   = 14
  if ((ntok != 13 && ntok != 14) || strcmp(tok[0], "st") != 0 || strcmp(tok[ntok - 1], "end") != 0) {
    return;
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

 int arr_base = 8; // 배열 시작 인덱스 (레거시 기준)
 /* 스칼라 #8: target_temp_c (신규) */
 if (ntok == 14) {               // 신규 프레임이면
    np.target_temp_c = (float)strtof(tok[8], NULL);
    arr_base = 9;                 // 배열 시작 위치 1칸 뒤로
  }
 // 레거시(13토큰)의 경우 np.target_temp_c는 기존값 유지



  /* 계수(배열) 4묶음 파싱 및 길이 검증 */
  double yt_tmp[2]; int y1d_n, y2c_n, y3c_n, yt_n;
  parse_csv_vec(tok[arr_base+0],  np.y1_den,      6, &y1d_n);
  parse_csv_vec(tok[arr_base+1],  np.y2_coeffs,   6, &y2c_n);
  parse_csv_vec(tok[arr_base+2],  np.y3_coeffs,   6, &y3c_n);
  parse_csv_vec(tok[arr_base+3],  yt_tmp,         2, &yt_n);


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


  // ▼▼▼ [이곳에 온도 관련 목표값 설정 후 실제 작동하는 로직을 위해서 필요] ▼▼▼
  // PC에서 받은 목표 온도를 실제 온도 컨트롤러(DAC)에 적용합니다.
    // g_params.target_temp_c가 유효한 값(0도 이상)일 경우에만 설정합니다. (기본값 -1.0f)
    if (g_params.target_temp_c >= 0.0f) {
        TC_Set_Target_Temperature(g_params.target_temp_c);
    }
  // ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲


}


/* ==========================================================
 * 송신 (PCB -> PC) (프로토콜 준수 확인)
 * ==========================================================
 * DSP 파이프라인 연산이 완료된 최종 결과(Stage 3, 5, 7, 8, 9)를
 * PC로 전송하기 위한 프레임 문자열을 만들고 UART로 송신하는 함수입니다.
 * DSP_Process_Sample() 함수에서 g_send_flag가 1로 설정되면 main() 루프에서 호출됩니다.
 *
 * [통신 프로토콜 (PCB -> PC)]
 * - 형식: st|{메타 5개}|{페이로드 1개(float 24개)}|end (총 29개 데이터 값)
 * - 구분자: 필드 '|', 페이로드 내부 ','
 * - 전체 프레임은 개행 문자 없이 한 줄(\r\n으로 종료)로 전송됩니다.
 *
 * - 메타 5필드 (순서 고정):
 * 1. block_count (uint32_t): 프레임 순번 카운터 (g_state.frame_sid)
 * 2. timestamp_ms (uint64_t): 전송 시점 타임스탬프 (ms) (HAL_GetTick())
 * 3. sampling_rate (float): *입력* 샘플링 속도 (kS/s) (g_params.sampling_rate / 1000.0f)
 * 4. block_size (uint16_t): *입력* 블록 크기 (샘플 수) (g_params.block_size)
 * 5. channel_mask (uint16_t): 활성 채널 마스크 (g_params.channel_mask)
 *
 * - 통합 페이로드 1필드 (float 24개, 순서 고정):
 * - [0 ~ 7]: RAW8 (Stage 3 Time Average 결과 8채널)
 * - [8 ~ 11]: RAVG4 (Stage 5 Ravg 결과 4채널)
 * - [12 ~ 23]: 채널별 상세 결과 (y2, y3, yt 순서 반복 x 4채널 Quad)
 * - [12~14]: y2_0, y3_0, yt_0 (Quad 0)
 * - [15~17]: y2_1, y3_1, yt_1 (Quad 1)
 * - [18~20]: y2_2, y3_2, yt_2 (Quad 2)
 * - [21~23]: y2_3, y3_3, yt_3 (Quad 3)
 *
 * - 데이터 소스: g_latest_results[24] 전역 배열 (DSP_Process_Sample() 함수에서 채워짐)
 * - 출력 형식: 모든 float 값은 소수점 6자리(%.6f)로 포맷됩니다.
 */
static void DSP_Send_Data_Frame(void)
{
  /* --- 1. 메타데이터 5개 준비 --- */
  // 1.1. block_count: 전송 프레임 순번. g_state 구조체에서 관리하며, 전송 시마다 1씩 증가.
  const uint32_t block_count = g_state.frame_sid++;

  // 1.2. timestamp_ms: 현재 시스템 시간(ms). HAL 라이브러리 함수 사용.
  const uint32_t timestamp_ms = rdv2_millis();

  // 1.3. sampling_rate: PC에서 설정받은 *입력* 샘플링 속도(g_params.sampling_rate, Hz 단위)를
  //     프로토콜 명세에 따라 kS/s 단위로 변환하여 사용.
  const float    sampling_rate_out_ksps = g_params.sampling_rate / 1000.0f;

  // 1.4. block_size: PC에서 설정받은 *입력* 블록 크기(g_params.block_size) 사용.
  const uint16_t block_size_in = (uint16_t)g_params.block_size;

  // 1.5. channel_mask: PC에서 설정받은 채널 마스크(g_params.channel_mask) 사용.
  const uint16_t channel_mask = (uint16_t)g_params.channel_mask;

  /* --- 2. 프레임 문자열 생성 (snprintf 사용) --- */
  // g_tx_line: 전송할 문자열을 담을 전역 버퍼 (크기: RDV2_TX_BUFSZ)
  // n: 현재까지 버퍼에 쓰여진 문자열 길이 (버퍼 오버플로우 방지용)




  /* 2.1. 헤더 생성: "st|메타5개|" */
  // %lu: unsigned long (block_count)
  // %llu: unsigned long long (timestamp_ms)
  // %.3f: float 소수점 3자리 (sampling_rate_out_ksps)
  // %u: unsigned int (block_size_in, channel_mask)


  int n = snprintf(g_tx_line, RDV2_TX_BUFSZ,
                   "st|%lu|%lu|%.3f|%u|%u|",  // sampling_rate 소수점 3자리까지 (kS/s)
                   (unsigned long)block_count,
                   (unsigned long)timestamp_ms,
                   sampling_rate_out_ksps,  // kS/s 단위 전송
                   (unsigned)block_size_in,
                   (unsigned)channel_mask);


  // snprintf 오류 또는 버퍼 오버플로우 시 함수 종료 (안전 코드)
  if (n < 0 || n >= RDV2_TX_BUFSZ) return;


  /* 2.2. 통합 페이로드 생성: "값1,값2,...,값24" */
  // g_latest_results[24] 배열에 저장된 DSP 최종 결과값을 순서대로 문자열 변환.
  // %.6f: float 소수점 6자리 형식 지정자.
  // 마지막 값(i=22)까지는 값 뒤에 콤마(,)를 붙임.

  for (int i = 0; i < 23; i++) {
    // 버퍼의 남은 공간(RDV2_TX_BUFSZ - n)에 문자열을 이어 붙임.
      n += snprintf(g_tx_line + n, RDV2_TX_BUFSZ - n, "%.6f,", g_latest_results[i]);

      // snprintf 오류 또는 버퍼 오버플로우 시 함수 종료
      if (n < 0 || n >= RDV2_TX_BUFSZ) return;
  }

  // 마지막 값(23)까지 페이로드를 마무리
  n += snprintf(g_tx_line + n, RDV2_TX_BUFSZ - n, "%.6f|", g_latest_results[23]);
  if (n < 0 || n >= RDV2_TX_BUFSZ) return;

  // ---- 신규 필드: 실제 온도(°C) 추가 ----
  n += snprintf(g_tx_line + n, RDV2_TX_BUFSZ - n, "%.2f|end\r\n", g_tempC);

  // snprintf 오류 또는 버퍼 오버플로우 시 함수 종료
  if (n < 0 || n >= RDV2_TX_BUFSZ) return;


  /* --- 3. UART 송신 --- */

  // 3.1. RS485 송신 모드 활성화 (DE 핀 HIGH)
  RDV2_DE_TX();


  // 3.2. UART3를 통해 g_tx_line 버퍼의 내용을 n 바이트만큼 전송 (타임아웃 100ms)
  // huart3: CubeMX에서 설정한 UART 핸들러 (실제 프로젝트에 맞게 확인 필요)
  HAL_UART_Transmit(&huart3, (uint8_t*)g_tx_line, (uint16_t)n, 100);


  // 3.3. RS485 수신 모드 복귀 (DE 핀 LOW)
  RDV2_DE_RX();
}

/* ==== UART Rx 콜백 ==== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    uint8_t c = g_rx_byte;

    // 1) 라인 버퍼에 누적
    if (g_rx_len < (RDV2_RX_BUFSZ - 1)) {
      g_rx_line[g_rx_len++] = (char)c;
      g_rx_line[g_rx_len]   = '\0';
    } else {
      // overflow → 리셋
      g_rx_len = 0;
      g_rx_line[0] = '\0';
    }

    // 2) 종료 조건: 개행 또는 "|end" 등장
    if (c == '\n' || (g_rx_len >= 4 && strstr(g_rx_line, "|end") != NULL)) {

      // 안전하게 로컬 복사 후 CR/LF 제거
      char line_buf[RDV2_RX_BUFSZ];
      size_t n = (g_rx_len < (RDV2_RX_BUFSZ - 1)) ? g_rx_len : (RDV2_RX_BUFSZ - 1);
      memcpy(line_buf, g_rx_line, n);
      line_buf[n] = '\0';

      // 우측 공백/CR/LF 제거
      for (int i = (int)n - 1; i >= 0; i--) {
        char ch = line_buf[i];
        if (ch == '\r' || ch == '\n' || ch == ' ' || ch == '\t') line_buf[i] = '\0';
        else break;
      }

      // 3) 새 파서 호출 (설정 프레임 반영)
      DSP_Parse_Settings(line_buf);

      // 4) 입력 버퍼 리셋
      g_rx_len = 0;
      g_rx_line[0] = '\0';
    }

    // 5) 다음 바이트 재개
    HAL_UART_Receive_IT(&huart3, &g_rx_byte, 1);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
//  MPU_Config();

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_FMC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  /* ==== [신규] 온도 제어 하드웨어 초기화 ==== */
    // 1. DAC 초기화 (SPI2, bsp_dac.c)
    if (BSP_DAC_Init() != HAL_OK) {
        // DAC 초기화 실패 처리
        // Error_Handler();
    }

    // 2. ADS1115 초기화 및 타이머 시작 (I2C1, TIM7, ads1115_process.c)
    // 2Hz로 ADS1115를 읽기 위한 타이머를 시작합니다.
    ADS1115_Process_Init();

    // 3. 초기 안전 온도 설정 (최소값으로 시작)
    TC_Set_Target_Temperature(TC_TEMP_SET_MIN);


  /* ==== SW_1 DSP: 초기화 ==== */
  memset(&g_params, 0, sizeof(dsp_params_t));
  // [단위] 내부 Hz 기준으로 초기화
  g_params.sampling_rate    = 50000.0f; // Hz
  g_params.target_rate      = 5.0f;    // Hz
  g_params.decim_rate       = (int)(g_params.sampling_rate / g_params.target_rate + 0.5f);
  g_params.movavg_r         = 1;
  g_params.movavg_ch        = 1;
  g_params.channel_mask     = 0xFF;
  g_params.block_size       = 2048; // 샘플 수
  g_params.target_temp_c    = -1.0f;   // 미설정(비활성) 디폴트 표기

  g_params.alpha=1.0; g_params.beta=1.0; g_params.gamma=1.0; g_params.k=10.0; g_params.b=0.0; g_params.r_abs=1;
  g_params.y1_num[0]=1.0; g_params.y1_num[1]=0.0; g_params.y1_num_len=2;


  /* --- y-chain 기본값을 항등식 계수로 수정 --- */

    // coeffs_y1 (y1_den): 분모를 상수 1로 설정
    g_params.y1_den[0] = 0.0; g_params.y1_den[1] = 0.0; g_params.y1_den[2] = 0.0;
    g_params.y1_den[3] = 0.0; g_params.y1_den[4] = 0.0; g_params.y1_den[5] = 1.0;
    g_params.y1_den_len = 6;

    // coeffs_y2 (y2_coeffs): y2 = y1 이 되도록 설정
    g_params.y2_coeffs[0] = 0.0; g_params.y2_coeffs[1] = 0.0; g_params.y2_coeffs[2] = 0.0;
    g_params.y2_coeffs[3] = 0.0; g_params.y2_coeffs[4] = 1.0; g_params.y2_coeffs[5] = 0.0;
    g_params.y2_coeffs_len = 6;

    // coeffs_y3 (y3_coeffs): y3 = y2 가 되도록 설정
    g_params.y3_coeffs[0] = 0.0; g_params.y3_coeffs[1] = 0.0; g_params.y3_coeffs[2] = 0.0;
    g_params.y3_coeffs[3] = 0.0; g_params.y3_coeffs[4] = 1.0; g_params.y3_coeffs[5] = 0.0;
    g_params.y3_coeffs_len = 6;

    // coeffs_yt (E, F): yt = y3 가 되도록 설정 (기존과 동일)
    g_params.E = 1.0; g_params.F = 0.0;


  DSP_Reset_State();
  memset(g_latest_results, 0, sizeof(g_latest_results));



  HAL_UART_Receive_IT(&huart3, &g_rx_byte, 1); // UART 수신 시작
  ADC_Process_Init(); // AD7606 시작
  RDV2_DE_RX();       // RS485 수신 모드
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
     DSP_Process_Sample(); // ADC 샘플 처리
         if (g_send_flag) {
           g_send_flag = 0;
           Update_Current_Temperature();  // <- 온도 전송 추가 (전송 직전 최신 온도 읽기)
           DSP_Send_Data_Frame(); // 결과 전송 + tempC 전송 추가

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
