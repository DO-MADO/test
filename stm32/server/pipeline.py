# ============================================================
# pipeline.py (Display-only, C-DSP integrated, 3-stream parser)
# - C(iio_reader) stdout: [1B type] + <II>(n_samp, n_ch) + float32[]
# - type: 1=STAGE3_8CH, 2=STAGE5_4CH(Ravg), 3=YT_4CH(final)
# - Python: 계산 없음. 수신 → JSON 직렬화 → WS 브로드캐스트.
# ------------------------------------------------------------
# ✅ pipeline.py 코드 설명
# • 이 파일은 "데이터 소스"에서 프레임을 읽어 와서, 웹소켓(WS) 클라이언트에게
# 전달할 준비를 하는 파이프라인 컨트롤러입니다.
# • 현재 기본 소스는 C로 작성된 iio_reader.exe(제드보드 경로)이고,
# Synthetic(가짜 데이터 발생기)도 선택 가능해요.
# • (향후) STM32로 전환 시에는 "SerialSource"를 추가해 같은 구조로 붙이면 됩니다.
#
# ✅ 핵심 포인트
# 1) SourceBase: "프레임 한 덩어리"를 읽어오는 공통 인터페이스
# 2) CProcSource: C 프로세스를 실행하고 stdout 바이너리를 파싱
# 3) SyntheticSource: 테스트/데모용 가짜 데이터 생성
# 4) Pipeline: Source에서 읽은 프레임을 가공 없이 큐에 넣고, app.py가 WS로 전송
#
# 📦 프레임 구조(바이너리)
# [1바이트: 타입] + [uint32 n_samp] + [uint32 n_ch] + [float32[n_samp*n_ch]]
# → 이걸 NumPy float32 (n_samp, n_ch) 2D 배열로 바꿔서 내부 처리합니다.
# ============================================================

from __future__ import annotations

import asyncio
import json
import struct
import subprocess
import threading
import time
import math
from dataclasses import dataclass, field 
from typing import Callable, Optional, List, Dict, Tuple
from serial_io import SerialLine, parse_dat_frame
import collections

import numpy as np
import sys



# -----------------------------
# [0] NaN/Inf 정규화 + strict JSON
# -----------------------------
# 목적: 그래프 라이브러리(Chart.js 등)는 NaN/Inf를 싫어합니다.
# JSON 직렬화 전에 NaN/Inf를 None(null)으로 바꾸고,
# numpy 타입을 파이썬 기본 타입으로 변환하여 안전하게 보냅니다.
#
# 사용처: 아래 _run()에서 payload 직렬화 전에 호출됨.

def _json_safe(v):
    """NaN/Inf를 None으로 바꾸고, numpy 타입/배열은 파이썬 내장형으로 변환."""
    if isinstance(v, dict):
        return {k: _json_safe(w) for k, w in v.items()}
    if isinstance(v, (list, tuple)):
        return [_json_safe(w) for w in v]
    if isinstance(v, np.ndarray):
        # float32 배열이라도 tolist 전에 비정상치 교체
        if np.issubdtype(v.dtype, np.floating):
            v = np.where(np.isfinite(v), v, np.nan)
        return _json_safe(v.tolist())
    if isinstance(v, (np.floating, np.integer)):
        v = float(v)
    if isinstance(v, float):
        # NaN/Inf → None (Chart.js는 null을 gap으로 처리)
        if not math.isfinite(v):
            return None
    return v


# -----------------------------
# [1] 공통 소스 베이스
# -----------------------------
# "데이터 소스"가 반드시 구현해야 하는 최소 인터페이스입니다.
#
# • read_frame(): 한 번 호출할 때마다 (ftype, arr) 한 프레임을 반환
# - ftype: 정수(프레임 타입)
# - arr: 2D NumPy float32 배열 (shape = [n_samp, n_ch])
# • terminate(): 소스 정리(프로세스 종료, 포트 닫기 등)

class SourceBase:
    def read_frame(self) -> Tuple[int, np.ndarray]:
        """한 번 호출에 '하나의 프레임' 반환: (ftype, arr [n_samp, n_ch], float32)."""
        raise NotImplementedError

    def terminate(self):
        pass



###################################################################################
# [Serial] 직렬 통신 소스 (PCB ↔ PC 텍스트 프레임)
# - 프로토콜: st|...|end  (파이프 구분자 '|', 배열 내부는 ',')
# - PCB → PC: DAT 프레임 6필드(메타5 + payload(24)) 수신 & 파싱
# - PC → PCB: 설정 프레임(11필드 압축형) 전송은 app.py에서 수행(_send_cfg_if_serial)
#
# 구조 개요
#   • SerialParams : 직렬 포트 관련 설정(수신 RX / 송신 TX)
#   • SerialSource : SourceBase 구현체. read_frame() 호출마다
#                    "S3 → S5 → Y2 → Y3 → YT" 5종 프레임을 순차적으로 반환.
#                    (UI는 이 순서를 기대하므로, 하나의 DAT 프레임을 5조각으로 분해)
#
# 데이터 흐름(수신)
#   SerialLine.read_frame()  →  parse_dat_frame()  →  deque에 5종 push  →  pop하여 반환
#
# 예외 처리
#   • RX 포트 미설정/오픈 실패: self.rx=None → 빈 프레임 반환(루프 유지)
#   • 파싱 오류(ValueError 등): 에러 로그만 찍고 빈 프레임 반환(루프 유지)
#
# 주의
#   • parse_dat_frame() 규격은 serial_io.py의 프로토콜 주석에 고정됨
#   • payload 길이 24, 메타 5 필드 고정. 불일치 시 ValueError
#   • 여기서는 “표시 전용”: 계산/보정 없음. 형식 변환과 큐잉만 수행
###################################################################################

@dataclass
class SerialParams:
    port: str = "COM11"            # 🔵 RX(수신)용 포트: PCB → PC 데이터 입력 (예: "COM11", "/dev/ttyUSB0")
    baud: int = 115200             #    RX 보레이트
    tx_port: Optional[str] = None  # 🔵 TX(송신)용 포트: PC → PCB 설정 프레임 전송 (필요 시 지정)
    tx_baud: int = 115200          #    TX 보레이트

class SerialSource(SourceBase):
    """
    RS485/RS232 텍스트 프레임을 읽어 5종의 '파이프라인 프레임'으로 분할해 반환.

    ▣ 한 번의 DAT 프레임(PCB→PC) → 내부적으로 다음 5개 프레임으로 쪼갬:
       1) FT_STAGE3   : raw8  (원시 8채널)          → shape (1, 8)
       2) FT_STAGE5   : ravg4 (R 도메인 4채널 평균) → shape (1, 4)
       3) FT_STAGE7_Y2: y2    (보정1, 4채널)       → shape (1, 4)
       4) FT_STAGE8_Y3: y3    (보정2, 4채널)       → shape (1, 4)
       5) FT_YT       : yt    (최종 출력 4채널)    → shape (1, 4)

    ▣ 반환 순서가 "S3→S5→Y2→Y3→YT" 인 이유:
       - pipeline._run() 에서 YT 수신 시점을 "한 세트 완료"로 보고 묶어서 WS로 내보내기 위함.
       - CProcSource/SyntheticSource 와 동일한 순서를 맞춰 UI 일관성 보장.
    """
    
    # 파이프라인 내 타입 상수(다른 소스들과 동일하게 맞춤)
    FT_STAGE3 = 0x01
    FT_STAGE5 = 0x02
    FT_YT     = 0x03
    FT_STAGE7_Y2 = 0x04
    FT_STAGE8_Y3 = 0x05

    def __init__(self, params: PipelineParams):
        # 앱/파이프라인 전체 파라미터 보관
        self.params = params
        
        # params.serial 이 없으면 기본 SerialParams 삽입 (안전장치)
        sp = getattr(params, "serial", None)
        if sp is None:
            sp = SerialParams()
            params.serial = sp

        # ---------- RX(수신) 포트 오픈 ----------
        # • RX 포트가 비어있거나("None"/""/"null") 잘못된 경우 → self.rx=None로 두고 빈 프레임 반환 루틴 사용
        self.rx = None
        if sp.port and str(sp.port).strip().lower() not in ("none", "", "null"):
            try:
                # SerialLine: 한 줄(st|...|end) 단위 프레임 추출기
                self.rx = SerialLine(sp.port, sp.baud)
                
            except Exception as e:
                # 오픈 실패해도 전체 파이프라인이 죽지 않도록 로그만 남기고 비활성화
                print(f"[SerialSource] RX open failed ({sp.port}): {e}", file=sys.stderr)
                self.rx = None


        # ---------- 프레임 출력 큐 ----------
        # • 한 번 수신한 DAT 프레임을 5조각으로 분할해 순서대로 내보내기 위해 사용
        self.frame_queue = collections.deque()


         # ---------- TX(송신) 포트 오픈(옵션) ----------
        # • PC→PCB 설정 프레임 전송은 app.py의 _send_cfg_if_serial()에서 self.tx 사용
        self.tx = None
        if sp.tx_port and str(sp.tx_port).strip().lower() not in ("none", "", "null"):
            try:
                self.tx = SerialLine(sp.tx_port, sp.tx_baud)
            except Exception as e:
                print(f"[SerialSource] TX open failed ({sp.tx_port}): {e}", file=sys.stderr)
                self.tx = None

    def terminate(self):
        """파이프라인 종료 시 포트 정리(에러는 조용히 무시)."""
        try: self.rx.close()
        except: pass
        try:
            if self.tx: self.tx.close()
        except: pass

    def read_frame(self) -> Tuple[int, np.ndarray]:
        """
        파이프라인 루프가 호출하는 단일 엔트리포인트.
        • 큐(frame_queue)에 이전에 쌓아둔 프레임이 있으면 먼저 반환
        • 없으면 RX로부터 'st|...|end' 한 줄을 읽어 parse_dat_frame()으로 파싱
        • 결과(raw8, ravg4, y2, y3, yt)를 5조각으로 큐에 push한 뒤, 첫 조각(S3)을 반환

        반환 형식: (ftype, arr)
          - ftype: 위 타입 상수들 중 하나
          - arr  : float32 2D, shape = (1, 채널수)
                   (CProcSource/SyntheticSource와 동일 shape 유지)
        """

        # 1) 큐에 남은 프레임이 있으면 바로 반환 → 소비 후 다음 호출에서 또 pop
        if self.frame_queue:
            return self.frame_queue.popleft()

        # 2) RX 미오픈(=수신 비활성화)인 경우: 파이프라인은 살아있어야 하므로 '빈 프레임' 반환
        if self.rx is None:
            time.sleep(0.01)  # CPU 과점유 방지
            return self.FT_YT, np.empty((0, 4), dtype=np.float32)

        # 3) 한 줄(st|...|end) 읽기 (타임아웃이면 None)
        line = self.rx.read_frame()

        # 4) 읽을 게 없으면(타임아웃): 빈 프레임 반환
        if line is None:
            time.sleep(0.001)
            return self.FT_YT, np.empty((0, 4), dtype=np.float32)

        try:
            # 5) 프로토콜 파싱
            #    meta(dict), raw8(8), ravg4(4), y2(4), y3(4), yt(4)
            meta, raw8, ravg4, y2, y3, yt = parse_dat_frame(line)
        except Exception as e:
            print(f"[SerialSource] Parse error: {e}", file=sys.stderr)
            # 파싱 실패해도 파이프라인이 멈추면 안 됨 → 로그만 남기고 빈 프레임
            return self.FT_YT, np.empty((0, 4), dtype=np.float32)


        # 6) 소스 간 shape 통일:
        #    - 시간축=1로 맞추고(한 줄 = 한 샘플 벡터), 채널축이 열 방향
        s3  = np.array([raw8],  dtype=np.float32)  # (1,8)
        s5  = np.array([ravg4], dtype=np.float32)  # (1,4)
        y2a = np.array([y2],    dtype=np.float32)  # (1,4)
        y3a = np.array([y3],    dtype=np.float32)  # (1,4)
        yta = np.array([yt],    dtype=np.float32)  # (1,4)

        # 7) “S3 → S5 → Y2 → Y3 → YT” 순으로 큐에 적재 후 첫 프레임 반환
        #    - pipeline._run()는 YT가 들어와야 한 세트를 완성으로 인식하므로 이 순서 유지 필수
        self.frame_queue.append((self.FT_STAGE3,    s3))
        self.frame_queue.append((self.FT_STAGE5,    s5))
        self.frame_queue.append((self.FT_STAGE7_Y2, y2a))
        self.frame_queue.append((self.FT_STAGE8_Y3, y3a))
        self.frame_queue.append((self.FT_YT,        yta))

        return self.frame_queue.popleft()


###################################################################################
###################################################################################




# -----------------------------
# [2] CProcSource — C 프로그램 실행 및 데이터 파싱
# -----------------------------
# 역할: C(iio_reader.exe)를 서브프로세스로 띄우고, 표준출력(stdout)으로
# 흘러나오는 바이너리 스트림을 "프레임 단위"로 읽어 파싱합니다.
# 주의: 여기서는 계산을 하지 않습니다(표시 전용 파이프라인). 계산은 C에서 끝남

class CProcSource(SourceBase):
    """
    iio_reader.c 프로세스를 실행하고, 표준 출력(stdout)으로 나오는
    데이터 스트림을 파싱하여 프레임 단위로 반환합니다.
    """
    FT_STAGE3 = 0x01  # 8ch (Stage3: 시간평균까지 끝난 원신호 블록)
    FT_STAGE5 = 0x02  # 4ch Ravg (Stage5)
    FT_YT     = 0x03  # 4ch 최종 yt
    FT_STAGE7_Y2 = 0x04  # y2 2차 함수 계산 처리 다항식함수 까지
    FT_STAGE8_Y3 = 0x05  # y3 2차 보정 처리 다항식 함수 또는 6번과 유사한 분수함수 까지

    def __init__(self, params: PipelineParams):
        # ❗ [최종 수정] C 프로그램에 전달할 6개 핵심 파라미터를 리스트로 구성
        # 순서/의미는 C측 main(argc, argv)에서 소비하는 규약과 일치해야 합니다.
        args = [
            params.exe_path, # 실행 파일 경로 (iio_reader.exe)
            params.ip, # 보드/장치 IP
            str(params.block_samples), # 블록 샘플 수
            str(int(params.sampling_frequency)), # 하드웨어 샘플레이트(Hz)
            str(params.target_rate_hz), # 타깃 출력 레이트(Hz)
            str(params.lpf_cutoff_hz), # LPF 컷오프(Hz)
            str(params.movavg_r), # R moving avg 길이
            str(params.movavg_ch), # ❗ CH moving avg 길이(추가)
            ]


        # C 리더(iio_reader.exe)를 실행
        self.proc = subprocess.Popen(
            args,
            stdin=subprocess.PIPE,  # 파이썬 → C 명령 전송용 (계수 업데이트 등)
            stdout=subprocess.PIPE, # C → 파이썬 데이터 스트림 수신용
            stderr=subprocess.PIPE,
            bufsize=0,              # 실시간 파이프 처리
        )


        # 표준 출력이 정상적으로 연결되었는지 확인합니다.
        if not self.proc.stdout:
            raise RuntimeError("CProcSource: C process stdout is not available.")
        if not self.proc.stdin: # ❗ stdin 연결 확인 (계수 전송 필요)
            raise RuntimeError("CProcSource: C process stdin is not available.")
        self._stdout = self.proc.stdout
        self._stdin = self.proc.stdin # ❗ 계수 업데이트용으로 보관
        self._hdr_struct = struct.Struct("<BII") # little-endian: uint8 + uint32 + uint32


    def _read_exact(self, n: int) -> bytes:
        """
        표준 출력에서 정확히 n 바이트를 읽어올 때까지 블로킹.
        스트림이 끊기거나 EOF가 오면 예외 발생.
        """
        buf = bytearray()
        while len(buf) < n:
            chunk = self._stdout.read(n - len(buf))
            if not chunk:
                # C 프로세스가 비정상 종료된 경우, stderr를 읽어 에러 메시지 힌트 제공
                stderr_output = self.proc.stderr.read().decode(errors='ignore')
                raise EOFError(f"CProcSource: unexpected EOF. Stderr: {stderr_output}")
            buf.extend(chunk)
        return bytes(buf)

    def read_frame(self) -> Tuple[int, np.ndarray]:
        """
        하나의 데이터 프레임(헤더 + 페이로드)을 읽고 파싱하여 반환합니다.
        파이프라인 메인 루프에서 계속 호출됩니다.
        """
        # 1) 헤더(9바이트) 읽기 → (ftype, n_samp, n_ch)
        hdr_bytes = self._read_exact(self._hdr_struct.size)
        ftype, n_samp, n_ch = self._hdr_struct.unpack(hdr_bytes)

        # 2) 페이로드(float32[n_samp*n_ch]) 읽기.
        payload_bytes = self._read_exact(n_samp * n_ch * 4)
        
        # 3) 바이트를 NumPy 2D 배열로 변환 (shape = [n_samp, n_ch])
        arr = np.frombuffer(payload_bytes, dtype=np.float32).reshape(n_samp, n_ch)
        
        return int(ftype), arr
    
    # ❗ [추가] C 프로세스에 커맨드를 보내는 메소드
    def send_command(self, line: str):
        """C 프로세스의 stdin으로 한 줄의 명령어를 보냅니다."""
        if self._stdin and not self._stdin.closed:
            try:
                # C에서 fgets로 읽을 수 있도록 개행 추가 + UTF-8 인코딩
                self._stdin.write(f"{line}\n".encode('utf-8'))
                self._stdin.flush()
            except (IOError, ValueError) as e:
                print(f"[pipeline] Failed to send command: {e}", file=sys.stderr)
    


    def terminate(self):
        """
        파이프라인이 중지될 때 C 프로세스를 안전하게 종료시킵니다.
        """
        try:
            self.proc.terminate()
        except Exception:
            pass


# -----------------------------
# [3] (옵션) SyntheticSource — 데모용
# -----------------------------
# 역할: 하드웨어 없이도 UI/WS 라인이 잘 작동하는지 빠르게 확인할 수 있게,
# 간단한 sin/cos 파형으로 3종 프레임을 번갈아 생성합니다.

class SyntheticSource(SourceBase):
    FT_STAGE3 = 0x01
    FT_STAGE5 = 0x02
    FT_YT     = 0x03
    FT_STAGE7_Y2 = 0x04
    FT_STAGE8_Y3 = 0x05

    def __init__(self, rate_hz: float = 10.0):
        self.rate = float(rate_hz)
        self._k = 0  # 1→2→3→1… 순환 인덱스
        
        _SLOW_DOWN_FACTOR = 5.0
        
        # SyntheticSource 모드 기다림 추가 (없으면 너무 많은 양을 보냄)
        try:
            self._sleep_duration = ((1.0 / self.rate) / 3.0) * _SLOW_DOWN_FACTOR
        except ZeroDivisionError:
            self._sleep_duration = 0.01


    def read_frame(self) -> Tuple[int, np.ndarray]:
         # SyntheticSource 모드 기다림 추가 (없으면 너무 많은 양을 보냄)
        time.sleep(self._sleep_duration)
        
        # 순서: STAGE3 (k=1) → STAGE5 (k=2) → Y2 (k=3) → Y3 (k=4) → YT (k=5) 입니다.
        self._k = (self._k % 5) + 1
        t = np.arange(5) / self.rate
        if self._k == 1:
            # 1. 첫 번째 순서(k=1)라면, STAGE3 (타입 1) 데이터를 만듭니다.
            # 8채널(c in range(8))짜리 sin 파형을 만듭니다.
            data = [np.sin(2*np.pi*(0.2 + 0.02*c)*t).astype(np.float32) for c in range(8)]
            arr = np.stack(data, axis=1)
            return self.FT_STAGE3, arr
        elif self._k == 2:
            # 2. 두 번째 순서(k=2)라면, STAGE5 (타입 2) 데이터를 만듭니다.
            # 4채널(c in range(4))짜리 cos 파형을 만듭니다.
            data = [np.cos(2*np.pi*(0.1 + 0.01*c)*t).astype(np.float32) for c in range(4)]
            arr = np.stack(data, axis=1)
            return self.FT_STAGE5, arr
        elif self._k == 3: 
            # 3. 세 번째 순서(k=3)라면, Y2 (타입 4) 데이터를 만듭니다.
            # (타입 번호(FT_STAGE7_Y2 = 0x04)와 순서(k=3)는 달라도 됩니다. 순서가 중요합니다.)
            data = [np.sin(2*np.pi*(0.15 + 0.01*c)*t + 0.2).astype(np.float32) for c in range(4)]
            arr = np.stack(data, axis=1)
            return self.FT_STAGE7_Y2, arr
        elif self._k == 4:
            # 4. 네 번째 순서(k=4)라면, Y3 (타입 5) 데이터를 만듭니다.
            data = [np.cos(2*np.pi*(0.15 + 0.01*c)*t + 0.4).astype(np.float32) for c in range(4)]
            arr = np.stack(data, axis=1)
            return self.FT_STAGE8_Y3, arr
        elif self._k == 5:
            # 5. 다섯 번째(k=5), 즉 '마지막' 순서라면, YT (타입 3) 데이터를 만듭니다.
            # C 코드와 마찬가지로 'YT' 프레임이 마지막에 와야 합니다.
            # 왜? pipeline.py의 _run() 루프는 YT가 들어와야 
            # "아, 5종 세트가 다 모였구나"라고 판단하고 웹소켓으로 전송하기 때문입니다.
            data = [np.sin(2*np.pi*(0.05 + 0.01*c)*t + 0.5).astype(np.float32) for c in range(4)]
            arr = np.stack(data, axis=1)
            return self.FT_YT, arr
        
        # [수정] self._k는 1~5만 가능하므로, 5에서 끝났기 때문에 else 블록은 필요 없습니다.
        # 만약의 버그(k=1~5가 아닌 경우)를 대비해 안전장치로 빈 값을 반환하게 할 수는 있습니다.
        return self.FT_YT, np.empty((0, 4), dtype=np.float32)


# -----------------------------
# [4] 파라미터 데이터 클래스 (최종 버전)
# -----------------------------
# 역할: 파이프라인 동작에 필요한 설정값(실행/표시/계수)을 한 곳에 보관합니다.
# app.py에서 생성/수정하여 Pipeline으로 전달합니다.

@dataclass
class PipelineParams:
# 실행 파라미터 -----------------------------------------
    mode: str = "cproc" # "cproc" | "synthetic" (※ STM32 전환 시 "serial" 추가 예정)
    exe_path: str = "iio_reader.exe" # C 실행 파일 경로
    ip: str = "192.168.1.133" # 장치 IP (제드보드 경로)
    block_samples: int = 16384 # 블록 크기
    sampling_frequency: int = 50000 # 하드웨어 샘플레이트(Hz)


    # DSP 파라미터 (Configuration 탭 연동) ------------------
    target_rate_hz: float = 5.0 # 타깃 출력 레이트(Hz)
    lpf_cutoff_hz: float = 2500.0 # LPF 컷오프(Hz)
    movavg_ch: int = 1 # ❗ CH MA(Smoothing) 길이. 1이면 사실상 OFF.
    movavg_r: int = 5 # R moving avg 길이
    
    # UI/메타 데이터 ----------------------------------------
    label_names: List[str] = field(default_factory=lambda: ["yt0", "yt1", "yt2", "yt3"]) # 4ch 라벨
    log_csv_path: Optional[str] = None
    
    # 4ch 탭 연동용 계수들 (C 코드의 기본값과 일치) ---------
    alpha: float = 1.0
    k: float = 10.0
    b: float = 0.0
    y1_den: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    y2_coeffs: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    y3_coeffs: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    E: float = 1.0
    F: float = 0.0


# -----------------------------
# [5] 파이프라인 클래스 (최종 수정 버전)
# -----------------------------
# 역할: 선택된 Source에서 프레임을 계속 읽어, 필요한 최소 가공 후
# app.py(WebSocket 루프)가 읽을 큐에 텍스트(JSON)로 넣습니다.
# (이 파일은 "표시 전용"이므로 복잡한 계산을 하지 않습니다.)

class Pipeline:
    """
    데이터 소스(C 또는 Synthetic)를 관리하고, 읽어온 데이터를 처리하여
    등록된 모든 웹소켓 컨슈머에게 브로드캐스트하는 메인 컨트롤러.
    """
    def __init__(self, params: PipelineParams, broadcast_fn: Callable[[Dict], None]):
        self.params = params
        self.broadcast_fn = broadcast_fn # app.py의 broadcast_fn을 직접 사용
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        
        # ❗ 데이터 수신을 시작한 시각(성능 측정용)
        self.start_time: Optional[float] = None

        # params.mode에 따라 데이터 소스 선택
        if self.params.mode == "cproc":
            self.src: SourceBase = CProcSource(self.params)
        elif self.params.mode == "synthetic":
            # SyntheticSource는 C와 달리 rate_hz만 필요
            self.src = SyntheticSource(rate_hz=self.params.target_rate_hz)
        elif self.params.mode == "serial":     # 시리얼 통신                  
            self.src = SerialSource(self.params)      # 시리얼 통신            
        else:
            # (향후) STM32용 SerialSource 추가 시 여기 분기 확장
            raise ValueError(f"Unknown mode: {self.params.mode}")


        # WebSocket 컨슈머(클라이언트) 목록 + 락
        self._consumers: List[asyncio.Queue[str]] = []
        self._consumers_lock = threading.Lock()


        # 최근 프레임/통계 캐시(WS payload 구성용)
        self._last_yt_time = None
        self._last_stats = None
        self._last_ravg = None
        self._last_y2   = None
        self._last_y3   = None
        self._last_yt   = None
        self._pending_stage3_block = None
        self._pending_ts = None
        
    
    # ❗ 계수 업데이트용 메서드 (재시작 없이 C에 반영)
    def update_coeffs(self, key: str, values: List[float]):
        """
        C-DSP 프로세스에 실시간으로 계수 업데이트 명령을 보냅니다.
        C 프로세스를 재시작하지 않습니다.
        
        사용 예)
        key="y1_den", values=[..., ..., ...]
        → C stdin으로 "y1_den v1,v2,v3,..." 형태로 전송
        """
        
        # 1) 파이썬 파라미터에도 동기화 (UI 반영/상태 보존)
        if hasattr(self.params, key):
            setattr(self.params, key, values)
        elif key == 'yt_coeffs' and len(values) == 2:
            self.params.E = values[0]
            self.params.F = values[1]

        # 2) C 프로세스로 전송할 문자열 생성
        # 예: "y1_den 0.0,0.0,1.0,0.0,0.0,0.0"
        values_str = ",".join(map(str, values))
        command = f"{key} {values_str}"

        # 3) 현재 소스가 CProcSource일 때만 실제 전송
        if isinstance(self.src, CProcSource):
            self.src.send_command(command)
            print(f"[Pipeline] Sent command to C: {command}")    



    def register_consumer(self) -> asyncio.Queue:
        """WS 루프(app.py)가 읽어갈 비동기 큐를 등록합니다."""
        q: asyncio.Queue[str] = asyncio.Queue(maxsize=2)
        with self._consumers_lock:
            self._consumers.append(q)
        return q

    def _broadcast(self, payload: dict):
        # 모든 컨슈머에게 JSON 메시지를 보내는 역할은 app.py가 담당합니다.
        # (과거 버전에서 사용되던 훅. 현재는 사용되지 않음)
        pass 

    def start(self):
        """백그라운드 스레드로 파이프라인 루프를 시작."""
        if self._thread and self._thread.is_alive(): return
        self._thread = threading.Thread(target=self._run, name="PipelineThread", daemon=True)
        self._thread.start()

    def stop(self):
        """파이프라인 정지 및 소스 종료."""
        self._stop.set()
        try: self.src.terminate()
        except Exception: pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)

    def _run(self):
        """메인 루프: 소스에서 프레임을 읽고, WS 큐로 전달할 payload를 구성."""
        while not self._stop.is_set():
            try:
                ftype, block = self.src.read_frame()
                
                # ❗ 첫 프레임 도착 시점 기록(성능 측정 등)
                if self.start_time is None and block.size > 0:
                    self.start_time = time.time()
                
            except EOFError: break   # 소스 종료(정상/비정상) 시 루프 탈출
            except Exception as e:
                print(f"[pipeline] read_frame error: {e}")
                break

            if block.size == 0: continue  # 아직 프레임이 누적 중이거나 빈 프레임이면 스킵
            now = time.time()
            n_samp, n_ch = block.shape
            

                
            # 타입별 마지막 캐시 갱신 --------------------------------------
            if ftype == CProcSource.FT_STAGE3:
                # Stage3(8ch) 블록은 yt가 올 때 함께 묶어서 보냄
                self._pending_stage3_block, self._pending_ts = block, now
                
                
            elif ftype == CProcSource.FT_STAGE5:
                # Ravg(4ch) 갱신: 그래프 그릴 때 사용할 시리즈 형태로 저장
                series = [block[:, k].tolist() for k in range(min(4, n_ch))]
                self._last_ravg = {"names": [f"Ravg{k}" for k in range(len(series))], "series": series}
            
            
            # (선택) 추가 프레임 타입: y2, y3 보정 단계
            elif ftype == CProcSource.FT_STAGE7_Y2:
                series = [block[:, k].tolist() for k in range(min(4, n_ch))]
                self._last_y2 = {"names": [f"y2_{k}" for k in range(len(series))], "series": series}
            elif ftype == CProcSource.FT_STAGE8_Y3:
                series = [block[:, k].tolist() for k in range(min(4, n_ch))]
                self._last_y3 = {"names": [f"y3_{k}" for k in range(len(series))], "series": series}    
                
                
            elif ftype == CProcSource.FT_YT:
                # 최종 4ch yt
                series = [block[:, k].tolist() for k in range(min(4, n_ch))]
                self._last_yt = {"names": self.params.label_names[:len(series)], "series": series}
                
                
                # 처리 통계 계산(블록당 처리 시간 기반)
                stats = None
                if self._last_yt_time is not None:
                    dt = max(1e-9, now - self._last_yt_time)
                    proc_sps_per_ch = n_samp / dt
                    stats = {
                        "sampling_frequency": float(self.params.sampling_frequency),
                        "block_samples": int(self.params.block_samples),
                        "actual_block_time_ms": float(dt * 1000.0),
                        "actual_blocks_per_sec": float(1.0 / dt),
                        "actual_proc_kSps": float(proc_sps_per_ch / 1000.0),
                        "actual_proc_Sps": float(proc_sps_per_ch),
                    }
                self._last_yt_time = now
                self._last_stats = stats


                # Stage3 블록을 yt와 묶어서 하나의 payload로 푸시 -----------------
                if self._pending_stage3_block is not None:
                    payload = {
                        "type": "frame", "ts": self._pending_ts,
                        "y_block": self._pending_stage3_block.tolist(),   # Stage3 원신호 블록
                        "n_ch": int(self._pending_stage3_block.shape[1]),
                        "block": {"n": int(self._pending_stage3_block.shape[0])},
                        "params": {"target_rate_hz": self.params.target_rate_hz},
                        "ravg_signals": self._last_ravg,
                        "stage7_y2": self._last_y2,
                        "stage8_y3": self._last_y3,
                        "derived": self._last_yt,   # 최종 yt
                        "stats": self._last_stats,  # 처리량/속도 지표
                    }
                    
                    # app.py의 WebSocket 루프가 사용할 수 있도록 텍스트(JSON)로 큐에 삽입
                    text = json.dumps(_json_safe(payload), separators=(",", ":"), allow_nan=False)
                    with self._consumers_lock:
                        for q in list(self._consumers):
                            try:
                                if q.full(): _ = q.get_nowait() # 최신만 유지
                                q.put_nowait(text)
                            except Exception: pass
                            
                    # 다음 묶음을 위해 Stage3 보류 블록 비우기
                    self._pending_stage3_block, self._pending_ts = None, None