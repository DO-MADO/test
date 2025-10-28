// ============================================================
//  iio_reader.c (A-option: 3 streams with frame_type)
//  - AD4858 via libiio
//  - DSP pipeline:
//      Stage3: LPF(SOS, DF2T) + TimeAverage  → 8ch
//      Stage5: R (log ratio) + MovingAvg     → 4ch (Ravg)
//      Stage9: y1 → y2 → y3 → yt             → 4ch (final)
//  - Output protocol (per frame):
//      [uint8 frame_type] + [block_hdr_t{n_samp,u32 n_ch}] + [float32 payload]
//  - frame_type: 1=STAGE3_8CH, 2=STAGE5_4CH, 3=STAGE9_YT_4CH
//  - Design: zero malloc/free in hot path, leak-free cleanup
// ============================================================

#include <iio.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <math.h>
#include <time.h>

#ifdef _WIN32
  #include <fcntl.h>
  #include <io.h>
  #include <windows.h>
#else
  #include <unistd.h>  
  #include <fcntl.h> 
  #include <termios.h>   //  추가: UART 설정용
  #include <errno.h>     //  추가: 오류 문자열용
  #include <sys/time.h>
#endif

// ---------- Block header (kept as before) ----------
#ifdef _MSC_VER
  #pragma pack(push, 1)
  typedef struct { uint32_t n_samp; uint32_t n_ch; } block_hdr_t;
  #pragma pack(pop)
#else
  typedef struct __attribute__((packed)) { uint32_t n_samp; uint32_t n_ch; } block_hdr_t;
#endif

// ---------- Frame type codes ----------
enum {
    FT_STAGE3_8CH = 1,
    FT_STAGE5_4CH = 2,
    FT_STAGE9_YT4 = 3,
    FT_STAGE7_Y2 = 4,
    FT_STAGE8_Y3 = 5
};



// ---------- Parameters (Python PipelineParams mirror) ----------
typedef struct {
    double sampling_frequency;     // ✅ Configuration: Hardware Sampling Rate (kS/s) 값 (PC→PCB) → AD4858 sampling_frequency 설정에 사용
    double target_rate_hz;         // ✅ Configuration: Target rate (S/s) 값 (PC→PCB) → decimation 계산에 사용
    double lpf_cutoff_hz;          // ✅ Configuration: LPF cutoff (Hz) 값 (PC→PCB) → 현재 버전에서는 문서화 용도로만 사용
    int    lpf_order;              // ✅ 내부 파라미터: SOS 섹션 수 계산용 (현재 4차 고정)
    int    movavg_r;               // ✅ Configuration: R Moving Avg (sec)→샘플 환산 값 (PC→PCB) → Stage5 이동 평균 길이
    // R = (alpha*beta*gamma) * log_k(sensor/standard) + b
    double alpha, beta, gamma, k, b;

    // y1 = P(r)/Q(r); y2 = poly(y1); y3 = poly(y2); yt = E*y3 + F
    double y1_num[10]; int y1_num_len;       // ✅ 4ch 탭: y1 분자 계수 (현재 r 항등식)
    double y1_den[10]; int y1_den_len;       // ✅ 4ch 탭: y1 분모 계수 [a5..a0] (PCB→PC 역방향 JSON과 매칭)
    double y2_coeffs[10]; int y2_coeffs_len; // ✅ 4ch 탭: y2 계수 [b5..b0]
    double y3_coeffs[10]; int y3_coeffs_len; // ✅ 4ch 탭: y3 계수 [c5..c0]
    double E, F;                             // ✅ 4ch 탭: yt 계수 [E,F]

    int    r_abs;                  // use fabs on inputs before ratio
} SignalParams;


// ---------- Runtime state ----------
typedef struct {
    double* lpf_state;     // ✅ LPF 상태버퍼: 채널별(SOS 섹션*2) DF2T 내부 상태
    float*  avg_tail;      // ✅ TimeAverage 테일 버퍼: decimation 나머지를 다음 블록으로 이월
    int     avg_tail_len;  // ✅ 현재 테일에 쌓인 샘플 수 (0..decim-1)
} ProcessingState;

// ---------- Helpers ----------
static int read_attr_str(struct iio_channel *ch, const char *attr, char *dst, size_t dst_len) {
    long n = iio_channel_attr_read(ch, attr, dst, dst_len);
    if (n <= 0) return 0;
    if ((size_t)n >= dst_len) n = (long)dst_len - 1;
    dst[n] = '\0';
    return 1;
}

static inline double polyval_f64(const double* c, int len, double x) {
    double r = 0.0;
    for (int i = 0; i < len; i++) r = r * x + c[i];
    return r;
}

// Simple centered moving average at TA rate
static void moving_average_f32(const float* in, float* out, int len, int N) {
        // ✅ CH Moving Avg (sec) → 샘플 수로 변환한 movavg_ch를 사용해 채널별 스무딩을 적용한다.
    //    Stage3 결과가 PC로 전달되기 전에 노이즈를 줄이는 용도이며, 최종적으로 ch0~ch7 데이터에 반영된다.
    if (N <= 1) { memcpy(out, in, (size_t)len * sizeof(float)); return; }
    const int half = N / 2;

    // 누적합 버퍼 (double 권장: 누적 오차↓)
    static double* psum = NULL;
    static int cap = 0;
    const int need = len + 1;
    if (cap < need) {
        free(psum);
        cap = need;
        psum = (double*)malloc((size_t)cap * sizeof(double));
        if (!psum) { // 메모리 부족 시 안전하게 원본 복사
            memcpy(out, in, (size_t)len * sizeof(float));
            return;
        }
    }

    // psum[i] = in[0] + ... + in[i-1], psum[0] = 0
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


// SOS DF2T in-place for single channel buffer x[len]
// state: [n_sections*2]
static void sos_df2t_inplace(float* x, int len, const double sos[][6], int n_sections, double* state) {
    for (int s = 0; s < n_sections; s++) {
        const double b0 = sos[s][0], b1 = sos[s][1], b2 = sos[s][2];
        const double a1 = sos[s][4], a2 = sos[s][5]; // a0 assumed 1
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



// ❗ [신규 추가] 문자열을 파싱하여 double 배열을 채우는 함수
static void parse_coeffs_from_string(const char* str, double* target_array, int max_len, int* actual_len) {
    // ✅ FastAPI ↔ C CLI 파이프라인이 실시간 계수 변경을 지원할 때 사용되는 헬퍼.
    //    "a,b,c" 형태 문자열을 double 배열로 변환하여 y1_den, y2_coeffs, y3_coeffs, [E,F] 갱신.
    int count = 0;
    char* buffer = strdup(str); // 원본 문자열 수정을 피하기 위해 복사
    char* token = strtok(buffer, ",");
    while (token != NULL && count < max_len) {
        target_array[count++] = atof(token);
        token = strtok(NULL, ",");
    }
    *actual_len = count;
    free(buffer);
}

// ❗ [신규 추가] stdin에서 커맨드를 읽고 처리하는 함수
static void check_and_process_stdin(SignalParams* P) {
    // ✅ PC에서 UART/STDIN으로 계수 업데이트 명령이 들어오는 경우에 대비한 루틴.
    //    예: "y1_den 1.0,0.5,..." → 4ch 탭 계수 (20개 항목) 중 일부를 런타임에 교체한다.
    char line[256];
    char key[64];
    char values_str[192];

#ifdef _WIN32
    HANDLE hStdin = GetStdHandle(STD_INPUT_HANDLE);
    DWORD bytes_avail = 0;
    if (!PeekNamedPipe(hStdin, NULL, 0, NULL, &bytes_avail, NULL)) return;
    if (bytes_avail == 0) return;
#else
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK); // ✅ 논블로킹 설정으로 DSP 루프가 멈추지 않도록 함
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
                double temp[2];
                int len;
                parse_coeffs_from_string(values_str, temp, 2, &len);
                if (len == 2) {
                    P->E = temp[0];
                    P->F = temp[1];
                }
            }
        }
    }
}


// ---------- UART helper ----------
#ifdef _WIN32
// Windows용 UART 핸들러 (HANDLE 타입 반환)
static HANDLE open_uart(const char *dev, int baud)
{
    HANDLE hSerial;
    char port_name[20];
    snprintf(port_name, sizeof(port_name), "\\\\.\\%s", dev);

    hSerial = CreateFile(
        port_name,
        GENERIC_READ | GENERIC_WRITE,
        0, 0,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        0
    );

    if (hSerial == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "ERR: cannot open %s (Error: %lu)\n", port_name, GetLastError());
        return INVALID_HANDLE_VALUE;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        fprintf(stderr, "ERR: GetCommState failed (Error: %lu)\n", GetLastError());
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    dcbSerialParams.BaudRate = CBR_115200;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;

    if(!SetCommState(hSerial, &dcbSerialParams)){
        fprintf(stderr, "ERR: SetCommState failed (Error: %lu)\n", GetLastError());
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    return hSerial;
}
#else
// 기존 Linux용 open_uart 코드는 그대로 둡니다.
static int open_uart(const char *dev, int baud)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "ERR: cannot open %s (%s)\n", dev, strerror(errno));
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "ERR: tcgetattr (%s)\n", strerror(errno));
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "ERR: tcsetattr (%s)\n", strerror(errno));
        close(fd);
        return -1;
    }

    return fd;
}
#endif



int main(int argc, char **argv) {
    // ============================================================
    //  ✨ SW 로직 한눈에 보기 (보고용 한글 주석)
    //  ● 입력: PC → CLI 인자로 전달되는 Configuration 6개 (block_samples, fs, target_rate,
    //          lpf_cutoff_hz, movavg_r, movavg_ch)
    //  ● 처리: AD4858 8채널 데이터를 Stage3 → Stage5 → Stage7/8/9 순으로 처리
    //      - Stage3 결과: ch0~ch7 (총 8개) → Raw Data (~Stage3)
    //      - Stage5 결과: Ravg0~Ravg3 (총 4개) → Raw Data (~Stage5)
    //      - Stage7/8 결과: yt?-y2 / yt?-y3 (각 4개씩, 총 8개)
    //      - Stage9 결과: yt0~yt3 (총 4개)
    //  ● 출력: frame_type + block_hdr_t + float32 payload 형식으로 FastAPI에 전송,
    //          사용자가 정리한 PCB→PC 50개 데이터와 1:1 매칭됨
    // ============================================================


    #ifdef _WIN32
        HANDLE uart_h = INVALID_HANDLE_VALUE;
    #else
        int uart_fd = -1;
    #endif

    #ifdef _WIN32
        uart_h = open_uart("COM3", 115200);
        if (uart_h != INVALID_HANDLE_VALUE) {
            fprintf(stderr, "[INFO] UART COM3 opened @115200\n");
        }
    #else
        uart_fd = open_uart("/dev/ttyPS0", 115200);
        if (uart_fd >= 0) {
            fprintf(stderr, "[INFO] UART /dev/ttyPS0 opened @115200\n");
        }
    #endif


    // ❗ [최종 수정] ---------- CLI (명령줄 인자 처리) ----------
    // ✅ FastAPI → C 실행파일 호출 시 전달되는 Configuration 6개 값을 그대로 수신한다.
    //    PC → PCB 26개 세팅 값 중 핵심 하드웨어 파라미터에 해당하며,
    //    이후 SignalParams 구조체에 저장되어 전 구간에서 참조된다.
    if (argc < 8) { // ❗ 인자 개수 8개로 수정
        fprintf(stderr, "Usage: %s <ip> <block> <fs> <target_rate> <lpf_cutoff> <movavg_r> <movavg_ch>\n", argv[0]);
        return 1;
    }
    const char *ip = argv[1];
    int block_samples = atoi(argv[2]);          // ✅ Configuration: Block Size (samples)
    long long sampling_freq = atoll(argv[3]);   // ✅ Configuration: Hardware Sampling Rate (kS/s)
    double target_rate_hz = atof(argv[4]);      // ✅ Configuration: Target rate (S/s)
    double lpf_cutoff_hz = atof(argv[5]);       // ✅ Configuration: LPF cutoff (Hz)
    int movavg_r = atoi(argv[6]);               // ✅ Configuration: R Moving Avg (sec) 환산값
    int movavg_ch = atoi(argv[7]); // ❗ CH MA(Smoothing) 인자 추가 → Configuration: CH Moving Avg (sec) 환산값
    const char *dev_name = "ad4858";

#ifdef _WIN32
    _setmode(_fileno(stdout), _O_BINARY);
#endif

    // ❗ [최종 수정] ---------- Parameters  ----------
    SignalParams P = {0};
    P.sampling_frequency = (double)sampling_freq; // ✅ Hardware Sampling Rate (kS/s) → PCB 내부 AD 변환 주파수
    P.target_rate_hz     = target_rate_hz;        // ✅ Target rate (S/s) → Stage3 TimeAverage decimation 계산
    P.lpf_cutoff_hz      = lpf_cutoff_hz;         // ✅ LPF cutoff (Hz) → SOS 설계 참고 정보 (현재 고정값)
    P.movavg_r           = movavg_r;              // ✅ R Moving Avg (sec→샘플) → Stage5 Ravg 필터 길이
    P.lpf_order          = 4; // LPF 차수는 4차로 고정

    // 나머지 계수들은 C 코드에 고정 (이 값들은 UI와 연동하지 않음)
    P.alpha=1.0; P.beta=1.0; P.gamma=1.0; P.k=10.0; P.b=0.0;
    // y1 분자 = r (항등식)
    P.y1_num[0] = 1.0;
    P.y1_num[1] = 0.0;
    P.y1_num_len = 2;
    P.y1_den[5]=1.0; P.y1_den_len=6;
    P.y2_coeffs[4]=1.0; P.y2_coeffs_len=6;
    P.y3_coeffs[4]=1.0; P.y3_coeffs_len=6;
    P.E=1.0; P.F=0.0;
    P.r_abs=1;

    // Precompute ratio/log constants
    const double base      = (P.k > 1.0) ? P.k : 10.0;    // ✅ 센서/표준 비율에 적용되는 로그 밑 (교정 상수)
    const double inv_log_b = 1.0 / log(base);             // ✅ log_k(x) 계산을 위한 보조 값
    const double r_scale   = (P.alpha * P.beta) * P.gamma; // ✅ αβγ 곱 → Stage5 R 계산 시 배율

    // ---------- IIO setup ----------
    struct iio_context *ctx = iio_create_network_context(ip); // ✅ 지정된 Zynq 보드(IP)의 libiio 컨텍스트 생성
    if (!ctx) { fprintf(stderr, "ERR: connect %s\n", ip); return 1; }
    struct iio_device *dev = iio_context_find_device(ctx, dev_name);
    if (!dev) { fprintf(stderr, "ERR: device '%s' not found\n", dev_name); iio_context_destroy(ctx); return 2; }

    if (sampling_freq > 0) {
        if (iio_device_attr_write_longlong(dev, "sampling_frequency", sampling_freq) < 0) {
            fprintf(stderr, "WARN: failed to set sampling_frequency\n");
        }
    }

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
        if (read_attr_str(ch, "scale", buf, sizeof(buf))) { s = atof(buf); }
        scales[n_in] = s;
        n_in++;
    }
    if (n_in < 8) { fprintf(stderr, "ERR: need >=8 inputs, got %d\n", n_in); free(in_ch); free(scales); iio_context_destroy(ctx); return 5; }
    const int n_ch = 8; // we will use first 8 channels as quad pairs

    // ---------- Buffer ----------
    struct iio_buffer *buf = iio_device_create_buffer(dev, (size_t)block_samples, false); // ✅ block_samples (PC 설정) 크기만큼 IIO 버퍼 확보
    if (!buf) { fprintf(stderr, "ERR: create buffer\n"); free(in_ch); free(scales); iio_context_destroy(ctx); return 6; }

    // ---------- DSP coeffs (SOS) & state ----------
    const int n_sections = 2; // ❗ 4를 2로 수정
    const double sos[2][6] = { // ❗ 4를 2로 수정
        {3.728052e-09, 7.456103e-09, 3.728052e-09, 1.000000e+00, -1.971149e+00, 9.713918e-01},
        {1.000000e+00, 2.000000e+00, 1.000000e+00, 1.000000e+00, -1.987805e+00, 9.880500e-01},
    };

    ProcessingState S = (ProcessingState){0};
    S.lpf_state = (double*)calloc((size_t)n_ch * (size_t)n_sections * 2, sizeof(double));
    if (!S.lpf_state) { fprintf(stderr, "ERR: alloc lpf_state\n"); iio_buffer_destroy(buf); free(in_ch); free(scales); iio_context_destroy(ctx); return 7; }

    const int decim = (int)(P.sampling_frequency / P.target_rate_hz); // ✅ Stage3 TimeAverage에 사용될 다운샘플 비율
    if (decim <= 0) { fprintf(stderr, "ERR: invalid decim\n"); free(S.lpf_state); iio_buffer_destroy(buf); free(in_ch); free(scales); iio_context_destroy(ctx); return 8; }
    S.avg_tail = (float*)calloc((size_t)decim * (size_t)n_ch, sizeof(float));
    if (!S.avg_tail) { fprintf(stderr, "ERR: alloc avg_tail\n"); free(S.lpf_state); iio_buffer_destroy(buf); free(in_ch); free(scales); iio_context_destroy(ctx); return 9; }
    S.avg_tail_len = 0;

    // ---------- Pre-allocate working buffers (no alloc in loop) ----------
    float *raw_f32  = (float*)malloc(sizeof(float) * (size_t)block_samples * (size_t)n_in);
    float *lpf_f32  = (float*)malloc(sizeof(float) * (size_t)block_samples * (size_t)n_in);
    float *ma_ch_out = (float*)malloc(sizeof(float) * (size_t)block_samples * (size_t)n_in); // ❗ CH MA 결과 버퍼 추가
    float *chan_buf = (float*)malloc(sizeof(float) * (size_t)block_samples);

    const int max_ta_out = block_samples / decim + 2;
    float *ta_combined = (float*)malloc(sizeof(float) * (size_t)(block_samples + decim) * (size_t)n_ch); // ✅ TimeAverage 계산 시 테일 + 현재 블록 결합 버퍼
    float *ta_out      = (float*)malloc(sizeof(float) * (size_t)max_ta_out * (size_t)n_ch);              // ✅ Stage3 결과 버퍼 (n_ta × 8채널)

    // Stage5/9 work arrays (TA rate)
    float *R_buf    = (float*)malloc(sizeof(float) * (size_t)max_ta_out);
    float *Ravg_buf = (float*)malloc(sizeof(float) * (size_t)max_ta_out);
    float *S5_out   = (float*)malloc(sizeof(float) * (size_t)max_ta_out * 4);
    float *Y2_out   = (float*)malloc(sizeof(float) * (size_t)max_ta_out * 4);
    float *Y3_out   = (float*)malloc(sizeof(float) * (size_t)max_ta_out * 4);
    float *YT_out   = (float*)malloc(sizeof(float) * (size_t)max_ta_out * 4);

    if (!raw_f32 || !lpf_f32 || !chan_buf || !ta_combined || !ta_out ||
        !R_buf || !Ravg_buf || !S5_out || !YT_out) {
        fprintf(stderr, "ERR: alloc work buffers\n");
        free(YT_out); free(S5_out); free(Ravg_buf); free(R_buf);
        free(ta_out); free(ta_combined); free(chan_buf); free(lpf_f32); free(raw_f32);
        free(S.avg_tail); free(S.lpf_state);
        iio_buffer_destroy(buf); free(in_ch); free(scales); iio_context_destroy(ctx);
        return 10;
    }

    // Quad mapping (sensor vs standard)
    const int sensor_idx[4]   = {0,2,4,6}; // ✅ Stage5 계산에서 센서 채널(ch0,2,4,6)
    const int standard_idx[4] = {1,3,5,7}; // ✅ Stage5 계산에서 표준 채널(ch1,3,5,7)

    // ---------- Main loop ----------
    for (;;) {
        check_and_process_stdin(&P); // ✅ 실시간 계수 업데이트 (y1/y2/y3/E/F) 처리 → 4ch 탭 20개 값 동기화
        if (iio_buffer_refill(buf) < 0) { fprintf(stderr, "ERR: buffer refill\n"); break; }

        // 1) raw → float (interleaved)
        //    → 결과는 PC로 송출되는 ch0~ch7 Stage3 데이터의 원본이 됨
        for (int ci = 0; ci < n_in; ci++) {
            struct iio_channel *ch = in_ch[ci];
            const uint8_t *p = (const uint8_t *)iio_buffer_first(buf, ch);
            const ptrdiff_t step = iio_buffer_step(buf);
            const double s = scales[ci];
            float *dst = raw_f32 + (size_t)ci;
            for (int k = 0; k < block_samples; k++) {
                int64_t v = 0; iio_channel_convert(ch, &v, p);
                *dst = (float)(v * s);
                dst += n_in;
                p   += step;
            }
        }

        // 2) LPF per channel (in-place over chan_buf) → lpf_f32
        //    → LPF cutoff (Hz) 설정에 대응, 고주파 노이즈 제거

        memcpy(lpf_f32, raw_f32, sizeof(float) * (size_t)block_samples * (size_t)n_in);

         for (int c = 0; c < n_ch; c++) {
             const float *src = raw_f32 + (size_t)c;
             for (int i = 0; i < block_samples; i++) chan_buf[i] = src[(size_t)i * (size_t)n_in];
             sos_df2t_inplace(chan_buf, block_samples, sos, n_sections, S.lpf_state + (size_t)c * (size_t)(n_sections*2));
             float *dst = lpf_f32 + (size_t)c;
             for (int i = 0; i < block_samples; i++) dst[(size_t)i * (size_t)n_in] = chan_buf[i];
         }

        // ❗ [추가] 2-2) Smoothing Filter (CH Moving Average) per channel
        //    → CH Moving Avg (sec) 설정 반영, Stage3 데이터 스무딩
        for (int c = 0; c < n_ch; c++) {
            const float *src = lpf_f32 + (size_t)c;
            for (int i = 0; i < block_samples; i++) chan_buf[i] = src[(size_t)i * (size_t)n_in];
            
            moving_average_f32(chan_buf, chan_buf, block_samples, movavg_ch); // ❗ 이동 평균 적용 (Configuration: CH Moving Avg)
            
            float *dst = ma_ch_out + (size_t)c;
            for (int i = 0; i < block_samples; i++) dst[(size_t)i * (size_t)n_in] = chan_buf[i];
        }

        // 3) TimeAverage (tail + current) → ta_out [n_ta x n_ch]
        //    → Target rate (S/s) 기준으로 다운샘플링, Raw Data (~Stage3) 8개 값 생성
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

        // ---- Stage3 frame emit (8ch) ----
        // ✅ PCB → PC 전달값: Raw Data (~Stage3) ch0~ch7 (총 8개). frame_type=1, payload=ta_out
        if (n_ta > 0) {
            uint8_t ft = (uint8_t)FT_STAGE3_8CH;
            block_hdr_t h3 = { (uint32_t)n_ta, (uint32_t)n_ch };
            fwrite(&ft, 1, 1, stdout);
            fwrite(&h3, sizeof(h3), 1, stdout);
            fwrite(ta_out, sizeof(float), (size_t)n_ta * (size_t)n_ch, stdout);
            fflush(stdout);
             // ✅ PC 수신 시: frame.y_block[row][col] → ch0~ch7 Raw Data (~Stage3) 그래프에 반영
        }

        // 4) Stage5 (Ravg 4ch) & Stage9 (YT 4ch)
        if (n_ta > 0) {
            // For each quad, compute R and Ravg, then y-chain and yt
            // ✅ sensor_idx / standard_idx 매핑: 각 쿼드(센서 vs 표준)로 Stage5~9 계산
            for (int q = 0; q < 4; q++) {
                const int si = sensor_idx[q];
                const int bi = standard_idx[q];

                // R at TA rate
                 // ✅ Stage5 계산용: ch0~ch7에서 센서/표준 비율 → 로그 스케일 → αβγ, b 적용
                for (int t = 0; t < n_ta; t++) {
                    double top = (double)ta_out[t * n_ch + si];
                    double bot = (double)ta_out[t * n_ch + bi];
                    if (P.r_abs) { if (top < 0) top = -top; if (bot < 0) bot = -bot; }
                    if (top < 1e-12) top = 1e-12;
                    if (bot < 1e-12) bot = 1e-12;
                    const double ratio = top / bot;
                    const double log_ratio = log(ratio) * inv_log_b; // log()/log(base)
                    R_buf[t] = (float)(r_scale * log_ratio + P.b);
                }
                // Ravg
                moving_average_f32(R_buf, Ravg_buf, n_ta, P.movavg_r); // ✅ R Moving Avg (sec→샘플) 설정이 여기서 사용됨

                // Stage5 output (Ravg)
                for (int t = 0; t < n_ta; t++) {
                    S5_out[t * 4 + q] = Ravg_buf[t];  // ✅ Raw Data (~Stage5) Ravg{q} 시퀀스 저장
                }

                // Stage9: y1..yt
                for (int t = 0; t < n_ta; t++) {
                    const double y1n = polyval_f64(P.y1_num, P.y1_num_len, r);   // ✅ y1 분자 계수 (UI: y1 분자)
                    const double y1d = polyval_f64(P.y1_den, P.y1_den_len, r);   // ✅ y1 분모 계수 [a5..a0]
                    const double y1  = y1n / ((fabs(y1d) < 1e-12) ? 1e-12 : y1d); // ✅ Stage7 이전 중간값
                    const double y2  = polyval_f64(P.y2_coeffs, P.y2_coeffs_len, y1); // ✅ y2 계수 [b5..b0]
                    const double y3  = polyval_f64(P.y3_coeffs, P.y3_coeffs_len, y2); // ✅ y3 계수 [c5..c0]
                    Y2_out[t * 4 + q] = (float)y2;              // ✅ Stage7: yt{q}-y2 파형 (PCB→PC 4개)
                    Y3_out[t * 4 + q] = (float)y3;              // ✅ Stage8: yt{q}-y3 파형 (PCB→PC 4개)
                    YT_out[t * 4 + q] = (float)(P.E * y3 + P.F); // ✅ Stage9: yt{q}-yt 파형 (최종 4개)
                }
            }

            // ---- Stage5 frame emit (4ch Ravg) ----
            {
                uint8_t ft = (uint8_t)FT_STAGE5_4CH;
                block_hdr_t h5 = { (uint32_t)n_ta, 4u };
                fwrite(&ft, 1, 1, stdout);
                fwrite(&h5, sizeof(h5), 1, stdout);
                fwrite(S5_out, sizeof(float), (size_t)n_ta * 4, stdout);
                fflush(stdout);
                 // ✅ PC 수신 시: WebSocket frame.ravg_signals.series[*][] → Ravg0~Ravg3 (Raw Data Stage5 4개 값)
            }

             // ❗ ---- [추가] Stage7 frame emit (4ch Y2) ----
            {
                uint8_t ft = (uint8_t)FT_STAGE7_Y2;
                block_hdr_t h7 = { (uint32_t)n_ta, 4u };
                fwrite(&ft, 1, 1, stdout);
                fwrite(&h7, sizeof(h7), 1, stdout);
                fwrite(Y2_out, sizeof(float), (size_t)n_ta * 4, stdout);
                fflush(stdout);
                // ✅ PC 수신 시: frame.stage7_y2.series[*][] → 4ch 탭 yt0-y2 ~ yt3-y2 (Stage7 4개 값)
                
            }

            // ❗ ---- [추가] Stage8 frame emit (4ch Y3) ----
            {
                uint8_t ft = (uint8_t)FT_STAGE8_Y3;
                block_hdr_t h8 = { (uint32_t)n_ta, 4u };
                fwrite(&ft, 1, 1, stdout);
                fwrite(&h8, sizeof(h8), 1, stdout);
                fwrite(Y3_out, sizeof(float), (size_t)n_ta * 4, stdout);
                fflush(stdout);
                // ✅ PC 수신 시: frame.stage8_y3.series[*][] → 4ch 탭 yt0-y3 ~ yt3-y3 (Stage8 4개 값)
            }

            // ---- Stage9 frame emit (4ch YT) ----
            {
                uint8_t ft = (uint8_t)FT_STAGE9_YT4;
                block_hdr_t h9 = { (uint32_t)n_ta, 4u };

                // stdout (Python)
                fwrite(&ft, 1, 1, stdout);
                fwrite(&h9, sizeof(h9), 1, stdout);
                fwrite(YT_out, sizeof(float), (size_t)n_ta * 4, stdout);
                fflush(stdout);
                 // ✅ PC 수신 시: frame.derived.series[*][] → 4ch 탭 yt0-yt ~ yt3-yt (Stage9 최종 4개 값)

            } // Stage9 emit block 종료

           // UART 로그 출력
            #ifdef _WIN32
                    if (uart_h != INVALID_HANDLE_VALUE) {
                        char buffer[256];
                        for (int t = 0; t < n_ta; t++) {
                            int len = snprintf(buffer, sizeof(buffer), "YT[%d] = %.3f, %.3f, %.3f, %.3f\r\n",
                                t, YT_out[t*4+0], YT_out[t*4+1], YT_out[t*4+2], YT_out[t*4+3]);
                            DWORD bytes_written;
                            WriteFile(uart_h, buffer, (DWORD)len, &bytes_written, NULL);
                        }
                    }
            #else
            
                    static FILE* logf = NULL;   // 실행 중 계속 유지되는 파일 포인터
                    if (uart_fd >= 0) {
                        for (int t = 0; t < n_ta; t++) {
                            struct timeval tv;
                            gettimeofday(&tv, NULL);

                            struct tm* tm_info = localtime(&tv.tv_sec);
                            char time_buf[64];
                            strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", tm_info);

                            // 1) UART 출력    
                            dprintf(uart_fd,
                                "%s.%03ld,%.3f,%.3f,%.3f,%.3f\r\n",
                                time_buf, tv.tv_usec / 1000,
                                YT_out[t*4+0], YT_out[t*4+1],
                                YT_out[t*4+2], YT_out[t*4+3]);

                            // 2) CSV 파일 저장 (보드 내부 log.csv)
                            if (!logf) {
                                // 프로그램 시작 시 한 번만 파일 열기
                                logf = fopen("yt_log.csv", "a");
                                if (!logf) {
                                    perror("fopen(yt_log.csv)");
                                }
                            }

                            if (logf) {
                                fprintf(logf, "%s.%03ld,%.3f,%.3f,%.3f,%.3f\n",
                                    time_buf, tv.tv_usec / 1000,
                                    YT_out[t*4+0], YT_out[t*4+1],
                                    YT_out[t*4+2], YT_out[t*4+3]);

                                fflush(logf);  // 버퍼에 쌓이지 않도록 즉시 디스크 반영
                            }
                        }
                    }
            #endif
           
        } 
    }    
    // ---------- Cleanup ----------
    free(Y3_out); free(Y2_out); free(YT_out); free(S5_out); free(Ravg_buf); free(R_buf);
    free(ta_out); free(ta_combined); free(chan_buf); free(lpf_f32); free(raw_f32);
    free(S.avg_tail); free(S.lpf_state);
    iio_buffer_destroy(buf);
    free(in_ch); free(scales);
    iio_context_destroy(ctx);
    // ✅ 위 정리는 메모리/디바이스 핸들 누수를 방지 → 장시간 운전 시 안정성 확보
    /* 추가: UART 닫기 */
    #ifdef _WIN32
    if (uart_h != INVALID_HANDLE_VALUE) CloseHandle(uart_h);
    #else
    if (uart_fd >= 0) close(uart_fd);
    #endif
    return 0;
}