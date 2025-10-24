# serial_io.py
# RS485/RS232 텍스트 프레임 파서 & 인코더 (st|...|end)
from __future__ import annotations
import serial, threading, time

SEP = "|"
ARRSEP = ","
ST = "st"
END = "end"

class SerialLine:
    """라인 단위 읽기/쓰기. 버퍼에서 st~end 사이 프레임만 추출."""
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.1):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self.buf = bytearray()
        self.lock = threading.Lock()

    def close(self):
        try: self.ser.close()
        except: pass

    def write_line(self, text: str):
        # 항상 '\n'로 종료
        if not text.endswith("\n"): text += "\n"
        self.ser.write(text.encode("utf-8"))
        self.ser.flush()

    def read_frame(self) -> str | None:
        """스트림에서 'st|'로 시작해 '|end'로 끝나는 한 프레임(문자열)을 반환"""
        chunk = self.ser.read(1024)
        if not chunk:
            return None
        with self.lock:
            self.buf.extend(chunk)
            data = self.buf.decode("utf-8", errors="ignore")
            # st의 시작과 end의 위치 탐색
            s = data.find("st" + SEP)
            e = data.find(SEP + "end")
            if s != -1 and e != -1 and e > s:
                frame = data[s:e+len(SEP+"end")]
                # 소비한 만큼 버퍼에서 제거
                remain = data[e+len(SEP+"end"):]
                self.buf = bytearray(remain.encode("utf-8"))
                return frame.strip()
        return None

# ---------------- PC -> PCB : CFG 26필드 인코더 ----------------
def encode_cfg(
    lpf_cutoff_hz: float,
    sampling_rate: float | int,
    target_rate: float,
    movavg_r: int,
    movavg_ch: int,
    channel_mask: int,     # 8채널 고정이면 255(0xFF)
    block_size: int,
    coeffs_y1: list[float],   # 6
    coeffs_y2: list[float],   # 6
    coeffs_y3: list[float],   # 6
    coeffs_yt: list[float],   # 2
) -> str:
    def arr(xs): return ARRSEP.join(str(x) for x in xs)
    fields = [
        ST,
        str(lpf_cutoff_hz),
        str(sampling_rate),
        str(target_rate),
        str(movavg_r),
        str(movavg_ch),
        str(channel_mask),
        str(block_size),
        arr(coeffs_y1),
        arr(coeffs_y2),
        arr(coeffs_y3),
        arr(coeffs_yt),
        END,
    ]
    # 시각화를 위해 개행을 넣던 예시는 실제 전송에선 한 줄이어야 하므로 join만 사용
    return SEP.join(fields)

# --------------- PCB -> PC : DAT(식별자 없음) 디코더 ---------------
def parse_dat_frame(frame: str):
    """
    입력 예:
    st|1234|1729145678123|1000|1024|255|0.10,0.08,...,0.44|end
      |  b0|      ts_ms     | fs |  N  | mask |     payload(24)      |
    반환: meta(dict) + payload(리스트 24개) + 분해된 서브 블록
    """
    if not frame.startswith("st|") or not frame.endswith("|end"):
        raise ValueError("Invalid frame markers")
    core = frame[3:-4]  # 'st|'와 '|end' 제거
    parts = core.split(SEP)
    if len(parts) != 6:   # 메타5 + payload1 = 6, (이미 'st' 제거됨)
        # parts=[block_count,timestamp_ms,sampling_rate,block_size,channel_mask,payload]
        # 총 6여야 정상. (위에서 3:-4 했으니 'st'와 'end'는 없음)
        raise ValueError(f"Invalid DAT fields: expected 6, got {len(parts)}")

    block_count    = int(parts[0])
    timestamp_ms   = int(parts[1])
    sampling_rate  = float(parts[2])
    block_size     = int(parts[3])
    channel_mask   = parts[4]
    payload_str    = parts[5]

    # payload 파싱
    payload = [float(x) for x in payload_str.split(ARRSEP) if x.strip() != ""]
    if len(payload) != 24:
        raise ValueError(f"Payload length must be 24, got {len(payload)}")

    # 채널우선 인덱싱 분해
    raw8 = payload[0:8]
    ravg4 = payload[8:12]
    ch0_y2, ch0_y3, ch0_yt = payload[12:15]
    ch1_y2, ch1_y3, ch1_yt = payload[15:18]
    ch2_y2, ch2_y3, ch2_yt = payload[18:21]
    ch3_y2, ch3_y3, ch3_yt = payload[21:24]

    stage7_y2 = [ch0_y2, ch1_y2, ch2_y2, ch3_y2]
    stage8_y3 = [ch0_y3, ch1_y3, ch2_y3, ch3_y3]
    yt4       = [ch0_yt, ch1_yt, ch2_yt, ch3_yt]

    meta = dict(
        block_count=block_count,
        timestamp_ms=timestamp_ms,
        sampling_rate=sampling_rate,
        block_size=block_size,
        channel_mask=channel_mask,
    )
    return meta, raw8, ravg4, stage7_y2, stage8_y3, yt4
