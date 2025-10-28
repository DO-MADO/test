# serial_io.py
# RS485/RS232 텍스트 프레임 파서 & 인코더 (st|...|end)
# -----------------------------------------------------------------------------
# [개요]
# - 이 모듈은 RS-232/RS-485 같은 직렬 통신으로 오가는 "텍스트 프레임"을 다룹니다.
# - 프레임은 항상 "st|"로 시작하고, "|end"로 끝납니다. (예: st|...|end)
#
# - 제공 기능
#   (1) SerialLine : 직렬 포트를 열고/닫고, 수신 버퍼에 데이터를 쌓아 두었다가
#                    "st|" ~ "|end" 사이의 '완전한 한 프레임'만 문자열로 꺼냄
#   (2) encode_cfg : PC -> PCB 로 보내는 설정(CFG) 프레임을 문자열로 생성
#   (3) parse_dat_frame : PCB -> PC 로 온 데이터(DAT) 프레임을 안전하게 파싱
#
# [텍스트 프레임 규칙]
#   - 필드 구분자: SEP = '|'
#   - payload 내부 숫자 구분자: ARRSEP = ','
#   - 시작 토큰: "st"  → 실제 프레임은 "st|"
#   - 종료 토큰: "end" → 실제 프레임은 "|end"
#
# [흐름도 - 수신 read_frame()]
#   직렬포트에서 바이트 읽기 → 내부 버퍼(bytearray)에 누적
#     → 버퍼를 UTF-8 문자열로 변환 → "st|" 위치 s / "|end" 위치 e 찾기
#     → (s!=-1 and e!=-1 and e>s) 이면 완전 프레임이므로 잘라서 반환
#     → 사용한 부분은 버퍼에서 제거, 나머지는 다음 수신에 활용
#
# [흐름도 - 송신 encode_cfg()]
#   입력 파라미터들을 문자열로 변환 → '|'로 이어붙임
#     → 맨 앞에 ST, 맨 뒤에 END 포함 → 한 줄 문자열로 반환
#
# [흐름도 - 파싱 parse_dat_frame()]
#   유효성 검사("st|"로 시작? "|end"로 끝?) → 양끝 토큰 제거 → '|'로 6필드 split
#     → 메타 필드들 형변환 → payload를 ','로 split하여 float 리스트(길이 24)
#     → 의미별 슬라이스(raw8, ravg4, ch별 y2/y3/yt) → 딕셔너리/리스트로 반환
# -----------------------------------------------------------------------------


from __future__ import annotations
import serial, threading, time


# ────*────*────*────*────*────* 프로토콜 상수 ───*────*────*────*────*────*
SEP = "|"      # 필드(열) 구분자. 예: st|필드1|필드2|...|end
ARRSEP = ","   # payload 내부의 여러 숫자를 나눌 때 사용되는 구분자. 예: 1.0,2.0,3.0
ST = "st"      # 시작 토큰(논리명). 실제 프레임은 항상 "st|"
END = "end"    # 종료 토큰(논리명). 실제 프레임은 항상 "|end"
# ────*────*────*────*────*────*───*────*────*────*────*────*────*────*────*



class SerialLine:
    """라인 단위 읽기/쓰기. 버퍼에서 st~end 사이 프레임만 추출.
    - 직렬 포트를 열어 두고, 수신 바이트를 내부 버퍼(self.buf)에 누적한다.
    - 누적된 문자열에서 'st|'와 '|end' 사이의 완전한 프레임을 찾아 문자열로 반환한다.
    """
    
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.1):
        # port: 'COM3' 또는 '/dev/ttyUSB0' 같은 실제 포트 경로/이름
        # baud: 통신 속도(bps). RS-485/232에서 흔히 쓰는 115200을 기본으로
        # timeout: read() 대기 시간(초). 0.1초 대기 후 데이터 없으면 빈 바이트 반환
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self.buf = bytearray()          # 수신 누적 버퍼(바이트 단위 저장)
        self.lock = threading.Lock()    # 멀티스레드 환경에서 버퍼 조작 보호용

    def close(self):
        # 직렬 포트를 닫는다. 종료 시 예외는 조용히 무시(테스트/종료 편의)
        try: self.ser.close()
        except: pass

    def write_line(self, text: str):
        # 항상 '\n'로 종료
        # - 송신은 "한 줄"로 처리하는 게 안전/로그 편의에 좋다.
        # - 만약 줄 끝에 개행이 없다면 붙여 준다.
        if not text.endswith("\n"): text += "\n"
        
        print(f"[TX LOG] Sending: {text.strip()}") # 터미널 로그에서 PC -> PCB 전송 확인는 로그
        
        self.ser.write(text.encode("utf-8"))    # 텍스트를 UTF-8 바이트로 변환하여 전송
        self.ser.flush()                        # OS/드라이버 버퍼로 즉시 밀어 넣기

    def read_frame(self) -> str | None:
        """스트림에서 'st|'로 시작해 '|end'로 끝나는 한 프레임(문자열)을 반환
        반환:
          - 완전한 프레임 문자열 (예: 'st|...|end') → 프레임 경계까지 도달한 경우
          - None → 아직 경계가 완성되지 않았거나 방금 읽은 바이트가 없는 경우
        """
        chunk = self.ser.read(1024)  # 이번 호출에서 새로 수신된 바이트(최대 1024)
        if not chunk:
            return None              # 아직 들어온 게 없으면 일단 None
        with self.lock:              # 버퍼 동시 접근을 막기 위해 락 사용
            self.buf.extend(chunk)   # 새 바이트를 내부 누적 버퍼에 추가
            
             # 누적 버퍼를 문자열로 변환. 깨진 바이트가 있어도 무시하고 이어서 해석
            data = self.buf.decode("utf-8", errors="ignore")  
            
            # st의 시작과 end의 위치 탐색
            s = data.find("st" + SEP)     # 'st|' 위치
            e = data.find(SEP + "end")    # '|end' 위치
            if s != -1 and e != -1 and e > s:
                # s~e 사이가 완전한 프레임 구간. 끝 토큰까지 포함하여 추출
                frame = data[s:e+len(SEP+"end")]
                
                # 소비한 만큼 버퍼에서 제거(프레임 뒤에 남은 꼬리 문자열만 유지)
                remain = data[e+len(SEP+"end"):]
                self.buf = bytearray(remain.encode("utf-8"))
                return frame.strip()    # 혹시 모를 앞뒤 공백 제거 후 반환
        return None   # 아직 프레임이 완성되지 않음


# ────*────*────*────*  PC -> PCB : 설정(Configuration) 프레임 11필드 인코더 ────*────*────*────*
"""
PC → PCB 설정 프레임은
'|' 기준 11필드(스칼라 7 + 배열 4)로 구성된 고정 프레임 구조이며,
각 배열 필드는 ',' 로 구분된 고정 길이 값(y1[6], y2[6], y3[6], yt[2])을 포함한다.
전체 데이터 값 개수는 27개다
"""

def validate_cfg_args(
    lpf_cutoff_hz, sampling_rate, target_rate,
    movavg_r, movavg_ch, channel_mask, block_size,
    coeffs_y1, coeffs_y2, coeffs_y3, coeffs_yt,
):
    """
    encode_cfg() 호출 전 인자 검증용 함수.
    목적:
      - 파라미터 타입/길이/범위를 미리 점검하여, 잘못된 설정 프레임 전송을 예방.
      - 펌웨어에서 NACK/오동작을 일으키는 값이 애초에 송신되지 않도록 방지.

    검증 항목 요약:
      1) 배열 길이 고정: y1=6, y2=6, y3=6, yt=2
      2) 숫자성(형변환 가능성): float/int 변환이 가능한지 사전 확인
      3) 기본 범위:
         - sampling_rate > 0
         - block_size > 0
         - 0 <= channel_mask <= 255  (8채널 고정 시 255 권장)

    예외:
      - 조건 불만족 시 ValueError 발생 → 호출부에서 try/except로 핸들링
    """
    
    # 1) 타입 및 길이 검증 (배열 길이 고정 규칙)
    if len(coeffs_y1) != 6 or len(coeffs_y2) != 6 or len(coeffs_y3) != 6 or len(coeffs_yt) != 2:
        raise ValueError("coeffs lengths must be y1=6, y2=6, y3=6, yt=2")


    # 2) 숫자형 변환 가능 여부 (형/문자 혼입 방지)
    _ = float(lpf_cutoff_hz); _ = float(sampling_rate); _ = float(target_rate)
    _ = int(movavg_r); _ = int(movavg_ch); _ = int(block_size)
    _ = int(channel_mask)   # 0~255 권장
    for arr in (coeffs_y1, coeffs_y2, coeffs_y3, coeffs_yt):
        for v in arr:
            _ = float(v)    # 각 계수가 숫자형으로 변환 가능한지 확인


    # 3) 값 범위(필요시 프로젝트 요구에 맞게 조정 가능)
    if not (0 < float(sampling_rate)):
        raise ValueError("sampling_rate must be > 0")
    if not (0 < int(block_size)):
        raise ValueError("block_size must be > 0")
    if not (0 <= int(channel_mask) <= 255):
        raise ValueError("channel_mask must be 0..255")


def encode_cfg(
    lpf_cutoff_hz: float,         # 저역통과필터 컷오프 주파수(Hz)
    sampling_rate: float | int,   # 샘플링 레이트(Hz)
    target_rate: float,           # (옵션) 리포트/다운샘플 목표 레이트(Hz)
    movavg_r: int,                # 이동평균 R 파라미터(예: 창 길이/계수)
    movavg_ch: int,               # 채널 평균 파라미터(예: 창 길이/계수)
    channel_mask: int,            # 8채널 고정이면 255(0xFF). 비트마스크
    block_size: int,              # 처리/전송 블록 크기(샘플 개수 등)
    coeffs_y1: list[float],       # y1 계산 계수 6개
    coeffs_y2: list[float],       # y2 계산 계수 6개
    coeffs_y3: list[float],       # y3 계산 계수 6개
    coeffs_yt: list[float],       # yt 계산 계수 2개
) -> str:
     # 내부 헬퍼: 숫자 리스트 → '1.0,2.0,3.0' 형태 문자열로 변환
    def arr(xs): return ARRSEP.join(str(x) for x in xs)
    
    # CFG 프레임은 "st| ... |end" 한 줄로 구성한다.
    # 나중에 write_line()에서 \n을 붙여 송신한다. (여기서는 개행 없이 join만)
    fields = [
        ST,                        # 시작 토큰(st)
        str(lpf_cutoff_hz),        # LPF 컷오프
        str(sampling_rate),        # 샘플링 레이트
        str(target_rate),          # 타깃 레이트
        str(movavg_r),             # 이동평균 R
        str(movavg_ch),            # 채널 평균
        str(channel_mask),         # 채널 마스크
        str(block_size),           # 블록 크기
        arr(coeffs_y1),            # y1 계수 6개를 콤마로 이어붙임
        arr(coeffs_y2),            # y2 계수 6개
        arr(coeffs_y3),            # y3 계수 6개
        arr(coeffs_yt),            # yt 계수 2개
        END,                       # 종료 토큰(end)
    ]
    # 시각화를 위해 개행을 넣던 예시는 실제 전송에선 한 줄이어야 하므로 join만 사용
    return SEP.join(fields)


# ────*────*────*────*────*────*───*────*────*────*────*────*────*────*────*────*────*────*

"""
PC -> PCB 예시 프레임 구조

st|6970.0|550.0|1270.0|2413|192|255|25600|
10.0,10.0,10.0,10.0,10.0,10.0|
0.0,0.0,0.0,0.0,1.0,0.0|
0.0,0.0,0.0,0.0,1.0,0.0|
55.0,23.0|end

파이프 필드 수: 11
(st / 7 스칼라 / 4 배열 / end)

배열 길이: y1=6, y2=6, y3=6, yt=2 (고정)

: 전체 데이터 값 개수는 27개
"""


# ────*────*────*────*────* PCB -> PC : 데이터(Data) 프레임 디코더 ────*────*────*────*────*
def parse_dat_frame(frame: str):
    """
        ─────────────────────────────────────────────
        📡 PCB -> PC : 데이터(Data) 프레임 구조 예시
        ─────────────────────────────────────────────

        ▶ 프레임 개요
            • 하나의 프레임은 'st' 로 시작하고 'end' 로 끝난다.
            • 필드 구분자는 '|' (파이프)
            • 배열 내부 구분자는 ',' (콤마)
            • 총 6필드 고정 (메타 5 + payload 1)
                [block_count, timestamp_ms, sampling_rate, block_size, channel_mask, payload(24개)]

        ─────────────────────────────────────────────
        ▶ 데이터 구성

        st|block_count|timestamp_ms|sampling_rate|block_size|channel_mask|payload(24)|end
            ↑              ↑             ↑             ↑            ↑             ↑
        (1) 블록번호   (2) 타임스탬프  (3) 샘플레이트 (4) 블록크기  (5) 활성채널  (6) 데이터(24개)

        ─────────────────────────────────────────────
        ▶ payload(24) 내부 구조 (고정 길이)

            ┌──────────────┬──────────────────────────────┐
            │ 구간(인덱스)   │ 설명                         │
            ├──────────────┼──────────────────────────────┤
            │  0 ~ 7       │ RAW8  (8채널 원시 값)         │
            │  8 ~ 11      │ RAVG4 (4채널 평균값)          │
            │ 12 ~ 14      │ ch0 : y2, y3, yt            │
            │ 15 ~ 17      │ ch1 : y2, y3, yt            │
            │ 18 ~ 20      │ ch2 : y2, y3, yt            │
            │ 21 ~ 23      │ ch3 : y2, y3, yt            │
            └──────────────┴──────────────────────────────┘
            → 총 24개의 float 고정

        ─────────────────────────────────────────────
        ▶ 실제 전송 예시

        st|1234|1729145678123|1000|1024|255|
        0.10,0.08,0.07,0.05,0.04,0.03,0.02,0.01,
        0.12,0.09,0.03,0.11,
        0.21,0.31,0.41,
        0.22,0.32,0.42,
        0.23,0.33,0.43,
        0.24,0.34,0.44|end

        ─────────────────────────────────────────────
        ▶ 구조 요약
            • 파이프 필드 수: 6
                (st / 5 메타 / 1 페이로드 / end)
            • payload 내부 길이: 24 (고정)
            • 총 데이터 값 수: 24 + 5 = 29개 (단, st/end 제외)
            • st, end 는 프레임 제어 토큰으로 데이터에 포함되지 않음
        ─────────────────────────────────────────────
    """
    
    # 1) 프레임 양끝 마커 확인: 'st|'로 시작 & '|end'로 끝나야 함
    if not frame.startswith("st|") or not frame.endswith("|end"):
        raise ValueError("Invalid frame markers")
    
    
    # 2) 양끝 마커 제거: 앞의 'st|' 3글자, 뒤의 '|end' 4글자
    core = frame[3:-4]  # 'st|'와 '|end' 제거
    
    
    # 3) 필드 쪼개기: '|' 기준으로 총 6개여야 정상
    parts = core.split(SEP)
    if len(parts) != 6:   # 메타5 + payload1 = 6, (이미 'st' 제거됨)
        # parts=[block_count,timestamp_ms,sampling_rate,block_size,channel_mask,payload]
        # 총 6여야 정상. (위에서 3:-4 했으니 'st'와 'end'는 없음)
        raise ValueError(f"Invalid DAT fields: expected 6, got {len(parts)}")


    # 4) 메타 필드 파싱/형변환
    block_count    = int(parts[0])
    timestamp_ms   = int(parts[1])
    sampling_rate  = float(parts[2])
    block_size     = int(parts[3])
    channel_mask   = parts[4]   # 문자열 그대로 둠(호출부에서 필요 시 int(...) 처리)
    payload_str    = parts[5]


    # 5) payload 파싱: 콤마로 나누어 float로 변환. 빈 항목은 제외
    payload = [float(x) for x in payload_str.split(ARRSEP) if x.strip() != ""]
    if len(payload) != 24:
        # 길이가 다르면 프로토콜 불일치 → 즉시 예외로 알려 문제를 빠르게 발견
        raise ValueError(f"Payload length must be 24, got {len(payload)}")


    # 6) 채널우선 인덱싱 분해 (의미대로 잘라서 사용하기 편하게)
    raw8 = payload[0:8]     # 원시 8채널 값
    ravg4 = payload[8:12]   # 평균/가공 4채널 값
    
    
    # ch0~ch3의 (y2, y3, yt) 3개씩
    ch0_y2, ch0_y3, ch0_yt = payload[12:15]
    ch1_y2, ch1_y3, ch1_yt = payload[15:18]
    ch2_y2, ch2_y3, ch2_yt = payload[18:21]
    ch3_y2, ch3_y3, ch3_yt = payload[21:24]
    
    
    # 스테이지별 묶음 형태로도 제공
    stage7_y2 = [ch0_y2, ch1_y2, ch2_y2, ch3_y2]
    stage8_y3 = [ch0_y3, ch1_y3, ch2_y3, ch3_y3]
    yt4       = [ch0_yt, ch1_yt, ch2_yt, ch3_yt]


    # 7) 메타 + 분해된 배열들을 튜플로 반환
    meta = dict(
        block_count=block_count,
        timestamp_ms=timestamp_ms,
        sampling_rate=sampling_rate,
        block_size=block_size,
        channel_mask=channel_mask,
    )
    return meta, raw8, ravg4, stage7_y2, stage8_y3, yt4


