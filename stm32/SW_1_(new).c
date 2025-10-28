\
/***********************************************************************
 * SW_1.c (주석 강화본) — "왕초보용" 단계별 해설 주석 추가
 *  - 절대 기존 코드 수정/삭제 없음 (주석만 추가)
 *  - 목적: ZedBoard 기반 10단계 DSP 파이프라인의 흐름을 한글로 쉽게 이해
 *  - 핵심 포인트
 *      (1) Raw 수집 → (2) 노이즈 필터 → (3) 시간평균/다운샘플 → (4) 로그비 R
 *      (5) 이동평균 Ravg → (6) 분수다항 y1 → (7) 다항 y2 → (8) 다항 y3
 *      (9) 최종 yt = E*y3+F → (10) Stage별 프레임 stdout + UART(선택)
 ***********************************************************************/

// ============================================================
//  zedboard_SW_pruned.c  —  10-Stage DSP Pipeline (trimmed)
//  Stages: (1)Raw → (2)LPF/Smooth → (3)TimeAvg → (4)R → (5)Ravg
//          (6)y1  → (7)y2         → (8)y3      → (9)yt → (10)Output
//  Frame: [uint8 frame_type] + [block_hdr_t{u32 n_samp, u32 n_ch}] + [float32 payload]
// ============================================================




// ****************************************************
// OLD 와 NEW의 차이는 주석 한글화 및 자세하게 NEW에서 추가한 차이
//       zedboard_SW 에서 추출한 연산 관련 부분 (로직,변수 등)
//                                  25. 10. 28 
// ****************************************************



#include <iio.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <math.h>

#ifdef _WIN32
  #include <fcntl.h>
  #include <io.h>
  #include <windows.h>
#else
  #include <unistd.h>
  #include <fcntl.h>
  #include <termios.h>
  #include <errno.h>
#endif

// ---------- Frame header ----------
#ifdef _MSC_VER
  /* [초보설명] block_hdr_t: 한 블록에 들어있는 샘플 수와 채널 수를 담는 헤더
   - n_samp: 이 프레임에 포함된 시간평균 결과의 샘플 개수
   - n_ch  : 이 프레임의 채널 개수(예: Stage3는 8, Stage5/7/8/9는 4)
*/
#pragma pack(push, 1)
  typedef struct { uint32_t n_samp; uint32_t n_ch; } block_hdr_t;
  #pragma pack(pop)
#else
  /* [초보설명] block_hdr_t: 한 블록에 들어있는 샘플 수와 채널 수를 담는 헤더
   - n_samp: 이 프레임에 포함된 시간평균 결과의 샘플 개수
   - n_ch  : 이 프레임의 채널 개수(예: Stage3는 8, Stage5/7/8/9는 4)
*/
typedef struct __attribute__((packed)) { uint32_t n_samp; uint32_t n_ch; } block_hdr_t;
#endif

// ---------- Frame type codes ----------
enum {
    FT_STAGE3_8CH = 1,   // TimeAverage(8ch)
    FT_STAGE5_4CH = 2,   // Ravg(4ch)
    FT_STAGE9_YT4 = 3,   // yt(4ch)
    FT_STAGE7_Y2  = 4,   // y2(4ch)
    FT_STAGE8_Y3  = 5    // y3(4ch)
};

// ---------- Parameters (runtime configurable for stages 4~9) ----------
typedef struct {
        /* [초보설명] 파라미터 묶음: 런타임/표준입력으로 일부 조정 가능 */
double sampling_frequency;   // HW Fs (samples/s) — stage (1)(3)
    double target_rate_hz;       // TA output rate (samples/s) — stage (3)
    double lpf_cutoff_hz;        // doc only (SOS fixed in this trimmed code)
    int    lpf_order;

    int    movavg_r;             // window (samples) for Ravg — stage (5)

    // R = (alpha*beta*gamma) * log_k(sensor/standard) + b
    double alpha, beta, gamma, k, b;
    int    r_abs;

    // y1 = P(r)/Q(r); y2 = poly(y1); y3 = poly(y2); yt = E*y3 + F
    double y1_num[10]; int y1_num_len;
    double y1_den[10]; int y1_den_len;
    double y2_coeffs[10]; int y2_coeffs_len;
    double y3_coeffs[10]; int y3_coeffs_len;
    double E, F;
} SignalParams;

// ---------- Runtime state ----------
/* [초보설명] ProcessingState: 필터 상태 및 시간평균 꼬리 버퍼 등 런타임 상태 보관 */
typedef struct {
    double* lpf_state;   // stage (2)
    float*  avg_tail;    // stage (3)
    int     avg_tail_len;
} ProcessingState;

// ---------- Small helpers ----------
static int read_attr_str(struct iio_channel *ch, const char *attr, char *dst, size_t dst_len) {
    long n = iio_channel_attr_read(ch, attr, dst, dst_len);
    if (n <= 0) return 0;
    if ((size_t)n >= dst_len) n = (long)dst_len - 1;
    dst[n] = '\0';
    return 1;
}

/* [초보설명] polyval_f64: 다항식 계수 배열 c[0..n-1]로 Horner 방식 평가 */
static inline double polyval_f64(const double* c, int len, double x) {
    double r = 0.0;
    for (int i = 0; i < len; i++) r = r * x + c[i];
    return r;
}

/* [초보설명] moving_average_f32: 간단 이동평균, N(윈도우) 크기 조절로 부드럽게 */
static void moving_average_f32(const float* in, float* out, int len, int N) {
    if (N <= 1) { memcpy(out, in, (size_t)len * sizeof(float)); return; }
    const int half = N / 2;

    static double* psum = NULL;
    static int cap = 0;
    const int need = len + 1;
    if (cap < need) {
        free(psum);
        cap = need;
        psum = (double*)malloc((size_t)cap * sizeof(double));
        if (!psum) { memcpy(out, in, (size_t)len * sizeof(float)); return; }
    }
    psum[0] = 0.0;
    for (int i = 0; i < len; i++) psum[i + 1] = psum[i] + (double)in[i];

    for (int i = 0; i < len; i++) {
        int start = i - half;
        int end   = i + (N - 1 - half);
        if (start < 0) start = 0;
        if (end >= len) end = len - 1;
        const int cnt = end - start + 1;
        const double sum = psum[end + 1] - psum[start];
        out[i] = (float)(sum / (double)cnt);
    }
}

// SOS DF2T (stage 2)
/* [초보설명] sos_df2t_inplace: 2차 필터(SOS) Direct Form II Transposed 구현 */
static void sos_df2t_inplace(float* x, int len, const double sos[][6], int n_sections, double* state) {
    for (int s = 0; s < n_sections; s++) {
        const double b0 = sos[s][0], b1 = sos[s][1], b2 = sos[s][2];
        const double a1 = sos[s][4], a2 = sos[s][5];
        double z1 = state[s*2 + 0], z2 = state[s*2 + 1];
        for (int i = 0; i < len; i++) {
            const double xi = x[i];
            const double yi = b0 * xi + z1;
            z1 = b1 * xi - a1 * yi + z2;
            z2 = b2 * xi - a2 * yi;
            x[i] = (float)yi;
        }
        state[s*2 + 0] = z1;
        state[s*2 + 1] = z2;
    }
}

// ---------- Runtime coeff update via stdin (stages 6~9 adjustability) ----------
/* [초보설명] parse_coeffs_from_string: '1,2,3' 문자열을 실수 배열로 파싱 */
static void parse_coeffs_from_string(const char* str, double* target_array, int max_len, int* actual_len) {
    int count = 0;
    char* buffer = strdup(str);
    char* token = strtok(buffer, ",");
    while (token && count < max_len) {
        target_array[count++] = atof(token);
        token = strtok(NULL, ",");
    }
    *actual_len = count;
    free(buffer);
}

/* [초보설명] check_and_process_stdin: 표준입력으로 보정계수(y1_den,y2,y3,yt) 실시간 적용 */
static void check_and_process_stdin(SignalParams* P) {
    char line[256], key[64], values_str[192];

#ifdef _WIN32
    HANDLE hStdin = GetStdHandle(STD_INPUT_HANDLE);
    DWORD bytes_avail = 0;
    if (!PeekNamedPipe(hStdin, NULL, 0, NULL, &bytes_avail, NULL)) return;
    if (bytes_avail == 0) return;
#else
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
#endif

    if (fgets(line, sizeof(line), stdin) != NULL) {
        if (sscanf(line, "%63s %191[^\n]", key, values_str) == 2) {
            if (strcmp(key, "y1_den") == 0) {
                parse_coeffs_from_string(values_str, P->y1_den, 10, &P->y1_den_len);
            } else if (strcmp(key, "y2_coeffs") == 0) {
                parse_coeffs_from_string(values_str, P->y2_coeffs, 10, &P->y2_coeffs_len);
            } else if (strcmp(key, "y3_coeffs") == 0) {
                parse_coeffs_from_string(values_str, P->y3_coeffs, 10, &P->y3_coeffs_len);
            } else if (strcmp(key, "yt_coeffs") == 0) {
                double tmp[2]; int n;
                parse_coeffs_from_string(values_str, tmp, 2, &n);
                if (n == 2) { P->E = tmp[0]; P->F = tmp[1]; }
            }
        }
    }
}

// ---------- UART open (stage 10 — RS232/RS485 path) ----------
#ifdef _WIN32
/* [초보설명] open_uart: 윈도우 COM 포트 오픈(115200,8N1) */
static HANDLE open_uart(const char *dev, int baud) {
    HANDLE hSerial; char port_name[20];
    snprintf(port_name, sizeof(port_name), "\\\\.\\%s", dev);
    hSerial = CreateFile(port_name, GENERIC_READ | GENERIC_WRITE, 0, 0,
                         OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (hSerial == INVALID_HANDLE_VALUE) return INVALID_HANDLE_VALUE;
    DCB dcb = {0}; dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(hSerial, &dcb)) { CloseHandle(hSerial); return INVALID_HANDLE_VALUE; }
    dcb.BaudRate = CBR_115200; dcb.ByteSize = 8; dcb.StopBits = ONESTOPBIT; dcb.Parity = NOPARITY;
    if (!SetCommState(hSerial, &dcb)) { CloseHandle(hSerial); return INVALID_HANDLE_VALUE; }
    return hSerial;
}
#else
/* [초보설명] open_uart: 리눅스 /dev/tty* 오픈(115200,8N1) */
static int open_uart(const char *dev, int baud) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) return -1;
    struct termios tty; if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }
    cfsetospeed(&tty, B115200); cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0; tty.c_oflag = 0; tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD); tty.c_cflag &= ~CSTOPB; tty.c_cflag &= ~CRTSCTS;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
    return fd;
}
#endif

/* [초보설명] main: 전체 파이프라인 엔트리 — 파라미터 파싱 → IIO 초기화 → 무한루프 DSP → 프레임 송출 */
int main(int argc, char **argv) {
#ifdef _WIN32
    _setmode(_fileno(stdout), _O_BINARY);
#endif
    /* [초보설명]
   실행 예: ./SW_1 <ip> <block_samples> <sampling_freq> <target_rate> <lpf_cutoff> <movavg_r> <movavg_ch>
   - block_samples: 한 번에 가져오는 raw 샘플 묶음 크기
   - sampling_freq: ADC 샘플레이트(예: 100000)
   - target_rate  : 시간평균 후 출력레이트(예: 10)
   - lpf_cutoff   : LPF 컷오프(본 파일 SOS는 고정, 문서용)
   - movavg_r     : R 이동평균 창 크기(샘플 단위)
   - movavg_ch    : 채널 스무딩 이동평균 창 크기(샘플 단위)
*/
if (argc < 8) {
        fprintf(stderr, "Usage: %s <ip> <block_samples> <sampling_freq> <target_rate> <lpf_cutoff> <movavg_r> <movavg_ch>\n", argv[0]);
        return 1;
    }

    // ---------- CLI / Params ----------
    const char *ip = argv[1];
    int block_samples = atoi(argv[2]);
    long long sampling_freq = atoll(argv[3]);
    double target_rate_hz = atof(argv[4]);
    double lpf_cutoff_hz = atof(argv[5]);
    int movavg_r = atoi(argv[6]);
    int movavg_ch = atoi(argv[7]);

    /* [초보설명] (4)~(9) 단계에 쓰이는 파라미터 기본값 세팅 */
SignalParams P = {0};
    P.sampling_frequency = (double)sampling_freq;
    P.target_rate_hz     = target_rate_hz;
    P.lpf_cutoff_hz      = lpf_cutoff_hz;
    P.lpf_order          = 4;
    P.movavg_r = movavg_r;

    // R params
    P.alpha=1.0; P.beta=1.0; P.gamma=1.0; P.k=10.0; P.b=0.0; P.r_abs=1;
    // y1/y2/y3/yt defaults
    P.y1_num[0]=1.0; P.y1_num[1]=0.0; P.y1_num_len=2;
    P.y1_den[5]=1.0; P.y1_den_len=6;
    P.y2_coeffs[4]=1.0; P.y2_coeffs_len=6;
    P.y3_coeffs[4]=1.0; P.y3_coeffs_len=6;
    P.E=1.0; P.F=0.0;

    const double base      = (P.k > 1.0) ? P.k : 10.0;
    const double inv_log_b = 1.0 / log(base);
    const double r_scale   = (P.alpha * P.beta) * P.gamma;

    // ---------- (1) IIO Context / Device / Channels ----------
    const char *dev_name = "ad4858";
    /* [초보설명] IIO 네트워크 컨텍스트 생성 (이더넷/IP로 AD4858 접근) */
struct iio_context *ctx = iio_create_network_context(ip);
    if (!ctx) { fprintf(stderr, "ERR: connect %s\n", ip); return 1; }
    struct iio_device *dev = iio_context_find_device(ctx, dev_name);
    if (!dev) { fprintf(stderr, "ERR: device '%s' not found\n", dev_name); iio_context_destroy(ctx); return 2; }

    if (sampling_freq > 0) iio_device_attr_write_longlong(dev, "sampling_frequency", sampling_freq);

    const int total_ch = iio_device_get_channels_count(dev);
    if (total_ch <= 0) { fprintf(stderr, "ERR: no channels\n"); iio_context_destroy(ctx); return 3; }

    struct iio_channel **in_ch = (struct iio_channel**)calloc((size_t)total_ch, sizeof(*in_ch));
    double *scales = (double*)calloc((size_t)total_ch, sizeof(double));
    if (!in_ch || !scales) { fprintf(stderr, "ERR: alloc in_ch/scales\n"); free(in_ch); free(scales); iio_context_destroy(ctx); return 4; }

    int n_in = 0;
    for (int i = 0; i < total_ch; i++) {
        struct iio_channel *ch = iio_device_get_channel(dev, i);
        if (!ch || iio_channel_is_output(ch)) continue;
        in_ch[n_in] = ch; iio_channel_enable(ch);
        double s = 1.0; char buf[64];
        if (read_attr_str(ch, "scale", buf, sizeof(buf))) s = atof(buf);
        scales[n_in++] = s;
    }
    if (n_in < 8) { fprintf(stderr, "ERR: need >=8 inputs, got %d\n", n_in); free(in_ch); free(scales); iio_context_destroy(ctx); return 5; }
    const int n_ch = 8;

    struct iio_buffer *buf = iio_device_create_buffer(dev, (size_t)block_samples, false);
    if (!buf) { fprintf(stderr, "ERR: create buffer\n"); free(in_ch); free(scales); iio_context_destroy(ctx); return 6; }

    // ---------- (2) LPF coeffs & state (SOS fixed) ----------
    /* [초보설명] SOS 2단(4차) LPF 사용 — 노이즈 제거 */
const int n_sections = 2;
    const double sos[2][6] = {
        {3.728052e-09, 7.456103e-09, 3.728052e-09, 1.0, -1.971149e+00, 9.713918e-01},
        {1.0,          2.0,          1.0,          1.0, -1.987805e+00, 9.880500e-01},
    };

    ProcessingState S = {0};
    S.lpf_state = (double*)calloc((size_t)n_ch * (size_t)n_sections * 2, sizeof(double));
    if (!S.lpf_state) { fprintf(stderr, "ERR: alloc lpf_state\n"); iio_buffer_destroy(buf); free(in_ch); free(scales); iio_context_destroy(ctx); return 7; }

    // ---------- (3) TimeAverage setup ----------
    /* [초보설명] (3) 시간평균/다운샘플 비율: decim = Fs/Frate (예: 100k/10=10000) */
const int decim = (int)(P.sampling_frequency / P.target_rate_hz);
    if (decim <= 0) { fprintf(stderr, "ERR: invalid decim\n"); free(S.lpf_state); iio_buffer_destroy(buf); free(in_ch); free(scales); iio_context_destroy(ctx); return 8; }
    S.avg_tail = (float*)calloc((size_t)decim * (size_t)n_ch, sizeof(float));
    if (!S.avg_tail) { fprintf(stderr, "ERR: alloc avg_tail\n"); free(S.lpf_state); iio_buffer_destroy(buf); free(in_ch); free(scales); iio_context_destroy(ctx); return 9; }
    S.avg_tail_len = 0;

    // ---------- Work buffers ----------
    /* [초보설명] 작업버퍼들: raw→lpf→ma_ch_out→ta_out→R/Ravg→y2/y3/yt */
float *raw_f32   = (float*)malloc(sizeof(float) * (size_t)block_samples * (size_t)n_in);
    float *lpf_f32   = (float*)malloc(sizeof(float) * (size_t)block_samples * (size_t)n_in);
    float *ma_ch_out = (float*)malloc(sizeof(float) * (size_t)block_samples * (size_t)n_in);
    float *chan_buf  = (float*)malloc(sizeof(float) * (size_t)block_samples);

    const int max_ta_out = block_samples / decim + 2;
    float *ta_combined = (float*)malloc(sizeof(float) * (size_t)(block_samples + decim) * (size_t)n_ch);
    float *ta_out      = (float*)malloc(sizeof(float) * (size_t)max_ta_out * (size_t)n_ch);

    float *R_buf    = (float*)malloc(sizeof(float) * (size_t)max_ta_out);
    float *Ravg_buf = (float*)malloc(sizeof(float) * (size_t)max_ta_out);
    float *S5_out   = (float*)malloc(sizeof(float) * (size_t)max_ta_out * 4);
    float *Y2_out   = (float*)malloc(sizeof(float) * (size_t)max_ta_out * 4);
    float *Y3_out   = (float*)malloc(sizeof(float) * (size_t)max_ta_out * 4);
    float *YT_out   = (float*)malloc(sizeof(float) * (size_t)max_ta_out * 4);

    if (!raw_f32 || !lpf_f32 || !ma_ch_out || !chan_buf || !ta_combined || !ta_out ||
        !R_buf || !Ravg_buf || !S5_out || !Y2_out || !Y3_out || !YT_out) {
        fprintf(stderr, "ERR: alloc work buffers\n");
        free(YT_out); free(Y3_out); free(Y2_out); free(S5_out); free(Ravg_buf); free(R_buf);
        free(ta_out); free(ta_combined); free(chan_buf); free(ma_ch_out); free(lpf_f32); free(raw_f32);
        free(S.avg_tail); free(S.lpf_state); iio_buffer_destroy(buf); free(in_ch); free(scales); iio_context_destroy(ctx);
        return 10;
    }

    // quads for R (stage 4)
    /* [초보설명] (4) 로그비 센서/스탠다드 쌍 매핑: (0/1), (2/3), (4/5), (6/7) */
const int sensor_idx[4]   = {0,2,4,6};
    const int standard_idx[4] = {1,3,5,7}; /* [초보설명] 위 쌍의 분모 채널들 */

    // ---------- UART (stage 10) ----------
#ifdef _WIN32
    HANDLE uart_h = open_uart("COM3", 115200);
#else
    int uart_fd = open_uart("/dev/ttyPS0", 115200);
#endif

    // ---------- Main DSP loop ----------
    /* [초보설명] === 메인 DSP 루프 시작 === */
for (;;) {
        check_and_process_stdin(&P); // (6)(7)(8)(9) 계수/계상 실시간 조정

        // (1) Raw Acquisition — ADC→float 변환 및 스케일 적용 → raw_f32 (interleaved by channel)
        if (iio_buffer_refill(buf) < 0) { fprintf(stderr, "ERR: buffer refill\n"); break; }
        for (int ci = 0; ci < n_in; ci++) {
            struct iio_channel *ch = in_ch[ci];
            const uint8_t *p = (const uint8_t *)iio_buffer_first(buf, ch);
            const ptrdiff_t step = iio_buffer_step(buf);
            const double s = scales[ci];
            float *dst = raw_f32 + (size_t)ci;
            for (int k = 0; k < block_samples; k++) {
                int64_t v = 0; iio_channel_convert(ch, &v, p);
                *dst = (float)(v * s);
                dst += n_in; p += step;
            }
        }

        // (2) LPF + Smoothing (per-channel) — 채널별 저역통과 + 이동평균으로 부드럽게
        memcpy(lpf_f32, raw_f32, sizeof(float) * (size_t)block_samples * (size_t)n_in);
        for (int c = 0; c < n_ch; c++) { // LPF
            const float *src = raw_f32 + (size_t)c;
            for (int i = 0; i < block_samples; i++) chan_buf[i] = src[(size_t)i * (size_t)n_in];
            sos_df2t_inplace(chan_buf, block_samples, sos, n_sections, S.lpf_state + (size_t)c * (size_t)(n_sections*2));
            float *dst = lpf_f32 + (size_t)c;
            for (int i = 0; i < block_samples; i++) dst[(size_t)i * (size_t)n_in] = chan_buf[i];
        }
        for (int c = 0; c < n_ch; c++) { // Smoothing (movavg_ch)
            const float *src = lpf_f32 + (size_t)c;
            for (int i = 0; i < block_samples; i++) chan_buf[i] = src[(size_t)i * (size_t)n_in];
            moving_average_f32(chan_buf, chan_buf, block_samples, movavg_ch);
            float *dst = ma_ch_out + (size_t)c;
            for (int i = 0; i < block_samples; i++) dst[(size_t)i * (size_t)n_in] = chan_buf[i];
        }

        // (3) Time Average / Decimation — decim 개씩 평균하여 출력속도 낮춤 → ta_out[n_ta * 8]
        const int total = S.avg_tail_len + block_samples;
        if (S.avg_tail_len > 0) {
            memcpy(ta_combined, S.avg_tail, (size_t)S.avg_tail_len * (size_t)n_ch * sizeof(float));
        }
        memcpy(ta_combined + (size_t)S.avg_tail_len * (size_t)n_ch,
               ma_ch_out, (size_t)block_samples * (size_t)n_ch * sizeof(float));

        const int n_ta = total / decim;
        const int rem  = total % decim;

        for (int o = 0; o < n_ta; o++) {
            for (int c = 0; c < n_ch; c++) {
                double acc = 0.0;
                const int base = o * decim;
                const float *ptr = ta_combined + (size_t)base * (size_t)n_ch + (size_t)c;
                for (int u = 0; u < decim; u++) acc += ptr[(size_t)u * (size_t)n_ch];
                ta_out[o * n_ch + c] = (float)(acc / (double)decim);
            }
        }
        S.avg_tail_len = rem;
        if (rem > 0) {
            memcpy(S.avg_tail,
                   ta_combined + (size_t)n_ta * (size_t)decim * (size_t)n_ch,
                   (size_t)rem * (size_t)n_ch * sizeof(float));
        }

        // ---- Stage3 frame emit (8ch) ---- — 시간평균 결과 8채널을 PC로 보냄
        //  PCB → PC 전달값: Raw Data (~Stage3) ch0~ch7 (총 8개). frame_type=1, payload=ta_out
        if (n_ta > 0) {
            uint8_t ft = (uint8_t)FT_STAGE3_8CH;
            block_hdr_t h3 = { (uint32_t)n_ta, (uint32_t)n_ch };
            fwrite(&ft, 1, 1, stdout);
            fwrite(&h3, sizeof(h3), 1, stdout);
            fwrite(ta_out, sizeof(float), (size_t)n_ta * (size_t)n_ch, stdout);
            fflush(stdout);
             //  PC 수신 시: frame.y_block[row][col] → ch0~ch7 Raw Data (~Stage3) 그래프에 반영
        }


        if (n_ta > 0) {
            // (4)(5)(6)(7)(8)(9)
            for (int q = 0; q < 4; q++) {
                const int si = sensor_idx[q];
                const int bi = standard_idx[q];

                // (4) R (log ratio)
                for (int t = 0; t < n_ta; t++) {
                    double top = (double)ta_out[t * n_ch + si];
                    double bot = (double)ta_out[t * n_ch + bi];
                    if (P.r_abs) { if (top < 0) top = -top; if (bot < 0) bot = -bot; }
                    if (top < 1e-12) top = 1e-12;
                    if (bot < 1e-12) bot = 1e-12;
                    const double ratio = top / bot;
                    const double log_ratio = log(ratio) * inv_log_b; // log_k
                    R_buf[t] = (float)(r_scale * log_ratio + P.b);
                }

                // (5) Ravg = MovingAverage(R) — R의 노이즈 저감
                moving_average_f32(R_buf, Ravg_buf, n_ta, P.movavg_r);
                for (int t = 0; t < n_ta; t++) S5_out[t * 4 + q] = Ravg_buf[t];

                // (6)(7)(8)(9) y-chain — y1(분수다항)→y2(다항)→y3(다항)→yt(E*y3+F)
                for (int t = 0; t < n_ta; t++) {
                    const double r_t = (double)Ravg_buf[t]; // <-- 버그 수정: y-chain 입력은 Ravg_buf[t]
                    const double y1n = polyval_f64(P.y1_num, P.y1_num_len, r_t);
                    const double y1d = polyval_f64(P.y1_den, P.y1_den_len, r_t);
                    const double y1  = y1n / ((fabs(y1d) < 1e-12) ? 1e-12 : y1d);
                    const double y2  = polyval_f64(P.y2_coeffs, P.y2_coeffs_len, y1);
                    const double y3  = polyval_f64(P.y3_coeffs, P.y3_coeffs_len, y2);
                    Y2_out[t * 4 + q] = (float)y2;
                    Y3_out[t * 4 + q] = (float)y3;
                    YT_out[t * 4 + q] = (float)(P.E * y3 + P.F);
                }
            }

            // --- Output Stage5: Ravg(4ch) --- — Ravg 4채널 프레임 송출
            {
                uint8_t ft = (uint8_t)FT_STAGE5_4CH;
                block_hdr_t h5 = { (uint32_t)n_ta, 4u };
                fwrite(&ft, 1, 1, stdout);
                fwrite(&h5, sizeof(h5), 1, stdout);
                fwrite(S5_out, sizeof(float), (size_t)n_ta * 4, stdout);
                fflush(stdout);
            }
            // --- Output Stage7: y2(4ch) --- — y2 4채널 프레임 송출
            {
                uint8_t ft = (uint8_t)FT_STAGE7_Y2;
                block_hdr_t h7 = { (uint32_t)n_ta, 4u };
                fwrite(&ft, 1, 1, stdout);
                fwrite(&h7, sizeof(h7), 1, stdout);
                fwrite(Y2_out, sizeof(float), (size_t)n_ta * 4, stdout);
                fflush(stdout);
            }
            // --- Output Stage8: y3(4ch) --- — y3 4채널 프레임 송출
            {
                uint8_t ft = (uint8_t)FT_STAGE8_Y3;
                block_hdr_t h8 = { (uint32_t)n_ta, 4u };
                fwrite(&ft, 1, 1, stdout);
                fwrite(&h8, sizeof(h8), 1, stdout);
                fwrite(Y3_out, sizeof(float), (size_t)n_ta * 4, stdout);
                fflush(stdout);
            }
            // --- Output Stage9: yt(4ch) --- — 최종 yt 4채널 프레임 송출
            {
                uint8_t ft = (uint8_t)FT_STAGE9_YT4;
                block_hdr_t h9 = { (uint32_t)n_ta, 4u };
                fwrite(&ft, 1, 1, stdout);
                fwrite(&h9, sizeof(h9), 1, stdout);
                fwrite(YT_out, sizeof(float), (size_t)n_ta * 4, stdout);
                fflush(stdout);
            }

            // (10) RS232/RS485 text stream (yt only, 4ch) — 선택적으로 직렬로 텍스트 로그
#ifdef _WIN32
            if (uart_h != INVALID_HANDLE_VALUE) {
                char line[256];
                for (int t = 0; t < n_ta; t++) {
                    int len = snprintf(line, sizeof(line), "YT[%d]=%.6f,%.6f,%.6f,%.6f\r\n",
                        t, YT_out[t*4+0], YT_out[t*4+1], YT_out[t*4+2], YT_out[t*4+3]);
                    DWORD bw; WriteFile(uart_h, line, (DWORD)len, &bw, NULL);
                }
            }
#else
            if (uart_fd >= 0) {
                for (int t = 0; t < n_ta; t++) {
                    dprintf(uart_fd, "%.6f,%.6f,%.6f,%.6f\r\n",
                            YT_out[t*4+0], YT_out[t*4+1], YT_out[t*4+2], YT_out[t*4+3]);
                }
            }
#endif
        }
    }

    // ---------- Cleanup ----------
#ifdef _WIN32
    if (uart_h && uart_h != INVALID_HANDLE_VALUE) CloseHandle(uart_h);
#else
    if (uart_fd >= 0) close(uart_fd);
#endif
    free(YT_out); free(Y3_out); free(Y2_out); free(S5_out); free(Ravg_buf); free(R_buf);
    free(ta_out); free(ta_combined); free(chan_buf); free(ma_ch_out); free(lpf_f32); free(raw_f32);
    free(S.avg_tail); free(S.lpf_state);
    iio_buffer_destroy(buf);
    free(in_ch); free(scales);
    iio_context_destroy(ctx);
    return 0;
}
