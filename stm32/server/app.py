# ============================================================
# app.py (FastAPI 웹 서버 + WebSocket 브로드캐스트)
# ------------------------------------------------------------
# ✅ app.py 코드 설명
# • 이 파일은 "웹 서버"입니다. FastAPI가 HTTP(REST)와 WebSocket(WS)을 제공합니다.
# • pipeline.py 가 만들어 주는 실시간 데이터 프레임을 받아서
# 웹 브라우저로 전송(WS)하고, 사용자가 바꾸는 설정(REST)을 받아서 파이프라인에 전달합니다.
# • 즉, app.py = "문 앞(서버)" / pipeline.py = "집 안(데이터 생산/중계)" 역할이라고 보면 됩니다.
#
# ✅ 핵심 포인트
# 1) 라우트(엔드포인트) 모음: /, /api/params, /api/coeffs, /api/save_data, /ws 등
# 2) WebSocket: 브라우저와 실시간 연결(그래프 업데이트를 위해 지속 연결)
# 3) pipeline.py 를 동적으로 import 하여 Pipeline 객체를 생성/보관(app.state)
# 4) 파라미터 변경 시 파이프라인 재시작(일부) 또는 계수 실시간 반영
# ============================================================



#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# -----------------------------
# [Imports] 외부/표준 라이브러리 불러오기
# -----------------------------
# • argparse: 명령줄 인자 파싱 (서버 시작 옵션)
# • json: JSON 직렬화/역직렬화
# • importlib.util: pipeline.py 동적 임포트에 사용
# • dataclasses.asdict: dataclass → dict 변환
# • pathlib.Path: 경로 다루기 (플랫폼 독립적)
# • sys: 모듈 등록/에러 출력 등
# • pandas: CSV 저장 등 데이터 가공에 사용
# • typing: 타입 힌트
# • fastapi.*: 라우팅/WS/응답 등
# • uvicorn: FastAPI 서버 실행기
# • pydantic.BaseModel: 요청/응답 모델 정의용
# • copy.deepcopy: 객체 복사
# • datetime, zoneinfo: 시간대 변환/타임스탬프 처리
# • time : 시간 관련

import argparse                     # 명령줄 인자 파싱 (서버 시작 옵션)
import json                         # JSON 직렬화/역직렬화
import importlib.util               # pipeline.py 동적 임포트에 사용
from dataclasses import asdict      # dataclass → dict 변환
from pathlib import Path            # 경로 다루기 (플랫폼 독립적) 
import sys                          # 모듈 등록/에러 출력 등
import pandas as pd                 # CSV 저장 등 데이터 가공에 사용
from typing import Optional, List   # 타입 힌트

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Response
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn                      # FastAPI 서버 실행기
from pydantic import BaseModel, Field   # 요청/응답 모델 정의용
from typing import Optional, List, Dict #  [추가] Dict 임포트
from copy import deepcopy               # 객체 복사
from datetime import datetime           # 시간대 변환/타임스탬프 처리
from zoneinfo import ZoneInfo           # 시간대 변환/타임스탬프 처리
import time
from serial_io import encode_cfg



# -----------------------------
# [Paths] 경로 상수 정의
# -----------------------------
# • ROOT: app.py가 위치한 폴더 경로
# • STATIC: 정적 파일(index.html 등)이 있는 폴더
# • PIPELINE_PATH: pipeline.py 파일 경로
# • COEFFS_JSON: 계수 저장용 JSON 파일 경로 (현재 코드에서 직접 사용되진 않음)

ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
PIPELINE_PATH = ROOT / "pipeline.py"
COEFFS_JSON = ROOT / "coeffs.json"


# -----------------------------
# [Dynamic Import] pipeline.py를 동적으로 임포트
# -----------------------------
# • spec_from_file_location → 모듈 스펙 생성
# • module_from_spec → 모듈 객체 생성
# • loader.exec_module → 실제로 로딩
# • 이렇게 하면 같은 폴더의 pipeline.py를 일반 import 없이 로드할 수 있습니다.

spec = importlib.util.spec_from_file_location("adc_pipeline", str(PIPELINE_PATH))
adc_pipeline = importlib.util.module_from_spec(spec)
sys.modules["adc_pipeline"] = adc_pipeline
assert spec.loader is not None
spec.loader.exec_module(adc_pipeline)

# pipeline.py 내부의 클래스/타입 alias
Pipeline = adc_pipeline.Pipeline
PipelineParams = adc_pipeline.PipelineParams


# -----------------------------
# [Pydantic 모델] 요청/응답 데이터 구조 정의
# -----------------------------
# • 프론트엔드에서 받는/보내는 JSON의 모양을 명확히 하기 위해 사용합니다.
# • Dataset/ChartData: 차트 데이터 포맷(Chart.js 구조)에 맞춤
# • AllChartData: 여러 스테이지의 차트를 한번에 받는 포맷
# • CoeffsUpdate: 계수 업데이트 요청(key, values)
# • ParamsIn: /api/params로 들어오는 변경 가능 파라미터(옵션 필드)


#  [추가] Chart.js의 단일 데이터셋 구조를 정확히 반영하는 모델
class Dataset(BaseModel):
    label: Optional[str] = None
    data: List[float]


#  [수정] ChartData 모델이 새로운 Dataset 모델을 사용하도록 변경
class ChartData(BaseModel):
    labels: List[float]
    datasets: List[Dataset] # List[Dict[...]] -> List[Dataset]


class AllChartData(BaseModel):
    stage3: ChartData
    stage5: ChartData
    stages789: Dict[str, Dict[str, ChartData]]


#  [수정] ParamsIn 모델을 아래와 같이 분리/수정
class CoeffsUpdate(BaseModel):
    key: str # 예: "y1_den", "y2_coeffs" 등
    values: List[float]
    
    
class ParamsIn(BaseModel):
    sampling_frequency: Optional[float] = None
    block_samples: Optional[int] = None
    target_rate_hz: Optional[float] = None
    lpf_cutoff_hz: Optional[float] = None
    movavg_ch_sec: Optional[float] = None
    movavg_r_sec: Optional[float] = None



# -----------------------------
# [FastAPI app & Helpers] 앱 생성 및 보조 함수
# -----------------------------
# • FastAPI 인스턴스 생성
# • /static 경로로 정적 파일 서빙(있을 경우)
# • _with_legacy_keys: 응답에 과거 키 이름을 추가(하위호환성 유지)

app = FastAPI(title="AD4858 Realtime Web UI")
if STATIC.exists():
    app.mount("/static", StaticFiles(directory=str(STATIC)), name="static")

def _with_legacy_keys(p: dict) -> dict:
    """하위호환을 위해 응답 dict에 과거 키 이름도 채워 넣습니다.
    예) y1_den → coeffs_y1, y2_coeffs → coeffs_y2 등
    프론트엔드에서 과거 키를 참고하는 코드가 있을 때 깨지지 않도록 보호합니다."""
    
    if "y1_den" in p: p["coeffs_y1"] = p["y1_den"]
    if "y2_coeffs" in p: p["coeffs_y2"] = p["y2_coeffs"]
    if "y3_coeffs" in p: p["coeffs_y3"] = p["y3_coeffs"]
    if "E" in p and "F" in p: p["coeffs_yt"] = [p["E"], p["F"]]
    return p


# -----------------------------
# [Routes] HTTP 엔드포인트들
# -----------------------------
# 1) GET / → index.html 반환 (정적 페이지)
# 2) POST /api/save_data → 프론트가 보낸 차트 데이터를 CSV로 저장
# 3) POST /api/coeffs → 계수 업데이트 (C 프로세스 stdin으로 실시간 반영)
# 4) GET /api/params → 현재 파라미터 조회
# 5) POST /api/params → 파라미터 업데이트(필요 시 파이프라인 재시작)
# 6) POST /api/params/reset → 기본값으로 리셋(파이프라인 재시작)
# 7) GET /favicon.ico → 파비콘 미제공(204)

@app.get("/")
async def index():
    return FileResponse(STATIC / "index.html")



# [추가] 데이터 처리 및 CSV 저장을 위한 헬퍼 함수
# - /api/save_data에서 호출됩니다.
# - 여러 차트 데이터를 시간축에 맞춰 합쳐 하나의 CSV로 저장합니다.
# - start_ts(Unix timestamp) 기준으로 labels(상대 시각)를 절대 시각으로 변환합니다.

def process_and_save_csv(all_data: AllChartData, file_path: Path, start_ts: float):
    """
    모든 차트 데이터를 병합하여 단일 CSV 파일로 저장합니다. (리샘플링 없음)
    • all_data: 프런트가 보내주는 전체 차트 데이터 묶음
    • file_path: 저장할 CSV 경로
    • start_ts: 데이터 시작 시각(Unix timestamp, float)
    """
    all_series = []
    
    # [수정] start_ts 인자를 받도록 시그니처 변경
    def create_series_from_chart_data(chart_data: ChartData, base_name: str, start_ts: float) -> list:
        series_list = []
        """ChartData → pandas.Series 목록으로 변환
        • labels: 상대 시각(초)
        • datasets: 각 시리즈의 값
        • base_name: 컬럼 접두어 (예: 'S3', 'S5', 'yt0_stage7')"""
        if not chart_data.labels or not chart_data.datasets:
            return []
            
        #  [1. 시간대 설정] 한국 시간(KST, UTC+9)을 정의합니다.
        kst = ZoneInfo("Asia/Seoul")
        
        num_datasets = len(chart_data.datasets)

        for i, ds in enumerate(chart_data.datasets):
            if not ds.data: continue
            
            # 상대 시간(label)을 절대 시간(Unix timestamp)으로 변환
            absolute_timestamps = [start_ts + label for label in chart_data.labels]
            #  [1. 시간대 설정] Unix timestamp를 KST 기준 datetime 객체로 변환
            datetime_index = [datetime.fromtimestamp(ts, tz=kst) for ts in absolute_timestamps]
            
            df = pd.DataFrame(index=datetime_index, data={'value': ds.data})
            
            #  [2. 1초 단위 로그 수집] 데이터를 1초 간격으로 리샘플링하고 평균값을 사용
            df = df.resample('1S').mean()
            
            if num_datasets > 1:
                col_name = f"{base_name}_{ds.label or i}"
            else:
                col_name = base_name

            df.rename(columns={'value': col_name}, inplace=True)
            series_list.append(df)
        return series_list

    #  [수정] create_series_from_chart_data 호출 시 start_ts 전달
    all_series.extend(create_series_from_chart_data(all_data.stage3, 'S3', start_ts))
    all_series.extend(create_series_from_chart_data(all_data.stage5, 'S5', start_ts))
    for ch, stages in all_data.stages789.items():
        for stage, data in stages.items():
            prefix = f"{ch}_{stage}"
            all_series.extend(create_series_from_chart_data(data, prefix, start_ts))
            
    if not all_series:
        return

    final_df = pd.concat(all_series, axis=1)
    final_df.sort_index(inplace=True)
    
    #  [추가] CSV로 저장하기 전, 인덱스(시간)의 형식을 원하는 문자열로 변경합니다.
    # '%Y-%m-%d %H:%M:%S.%f'는 마이크로초(6자리)까지 표시, .str[:-3]으로 밀리초(3자리)에서 자름
    final_df.index = final_df.index.strftime('%Y-%m-%d %H:%M:%S.%f').str[:-3]
    
    #  [수정] CSV 저장 시 인덱스 컬럼명을 'Timestamp'로 변경
    final_df.to_csv(file_path, float_format='%.6f', index_label='Timestamp')




# [추가] 데이터 저장을 위한 새로운 API 엔드포인트
# - 프론트에서 현재 차트 데이터를 AllChartData 형태로 보내면 CSV로 저장합니다.
# - 로그 경로는 ../../logs/날짜/log_data*.csv 에 순번 붙여 저장합니다.
@app.post("/api/save_data")
async def save_data(data: AllChartData):
    try:
        # --- 파일 경로 설정 (기존과 동일) ---
        log_base_dir = Path("../../logs")
        today_str = datetime.now().strftime('%Y.%m.%d')
        log_dir = log_base_dir / today_str
        log_dir.mkdir(parents=True, exist_ok=True)
        
        base_filename = "log_data"
        counter = 0
        while True:
            suffix = "" if counter == 0 else str(counter)
            file_path = log_dir / f"{base_filename}{suffix}.csv"
            if not file_path.exists():
                break
            counter += 1
            
        #  [추가] pipeline에서 데이터 시작 시간(Unix timestamp)을 가져옵니다.
        start_timestamp = app.state.pipeline.start_time
        if start_timestamp is None:
            # 데이터가 아직 수신되지 않은 경우, 현재 시간을 기준으로 합니다.
            start_timestamp = time.time()
            
        #  [수정] 데이터 처리 함수 호출 시 start_timestamp 전달
        process_and_save_csv(data, file_path, start_timestamp)
        
        return {"ok": True, "message": f"Data saved to {file_path}"}
    except Exception as e:
        print(f"[ERROR] Failed to save data: {e}")
        return {"ok": False, "message": str(e)}




# [신규 추가] 계수 업데이트를 위한 API 엔드포인트
# - 파이프라인(현재 CProcSource)에 실시간으로 계수를 반영합니다. (프로세스 재시작 없음)
# - /api/coeffs 로 {key, values} 를 보내면 pipeline.update_coeffs 가 처리합니다.
@app.post("/api/coeffs")
async def set_coeffs(p: CoeffsUpdate):
    """실행 중인 C 프로세스에 계수만 실시간으로 업데이트합니다."""
    app.state.pipeline.update_coeffs(p.key, p.values)

    # UI의 'Configuration' 탭 정보도 동기화
    updated_params = _with_legacy_keys(asdict(app.state.pipeline.params))
    return {
        "ok": True,
        "message": f"Coefficients for '{p.key}' updated.",
        "params": updated_params
    }



@app.get("/api/params")
async def get_params():
    #  .model_dump() -> asdict() 수정 (1/4)
    return _with_legacy_keys(asdict(app.state.pipeline.params))




@app.post("/api/params")
async def set_params(p: ParamsIn):
    """
    파라미터 업데이트 엔드포인트 (최종 버전)
    - UI의 모든 파라미터를 처리하고 단위를 변환합니다.
    - C 코드에 영향을 주는 파라미터 변경 시 파이프라인을 재시작하고
    'restarted' 신호를 보내 페이지 새로고침을 유도합니다.


    🔎 흐름 정리
    1) 프런트에서 바뀐 값들만 옵니다(옵션 필드). 예: {movavg_r_sec: 0.5}
    2) 초 단위 값을 샘플 개수로 바꿔야 하는 항목은 계산합니다.
    - CH MA(sec) → sampling_frequency 로 곱해서 샘플수
    - R MA(sec) → target_rate_hz 로 곱해서 샘플수
    3) 실제 변경이 있는지만 확인하고, 중요한 키가 바뀌면 파이프라인 재시작.
    (중요 키: fs, block_samples, target_rate_hz, lpf_cutoff_hz, movavg_r, movavg_ch)
    4) 재시작되면 새 Pipeline 인스턴스를 app.state.pipeline 에 등록하고, 클라이언트에게 알려줍니다.
    """
    
    # 1. UI로부터 받은 데이터 중 실제 값이 있는 것만 사전 형태로 추출
    body = p.model_dump(exclude_unset=True)
    
    # 2. 현재 파이프라인의 파라미터를 사전 형태로 복사
    current_params = app.state.pipeline.params
    new_params_dict = asdict(current_params)
    
    # 3. '초(sec)' 단위를 C가 사용할 '샘플 수'로 변환
    #    - 변환에 필요한 최신 주파수 값을 사용 (body에 있으면 body 값, 없으면 현재 값)
    fs = body.get("sampling_frequency", current_params.sampling_frequency)
    tr = body.get("target_rate_hz", current_params.target_rate_hz)

    if "movavg_ch_sec" in body:
        sec = body["movavg_ch_sec"]
        # CH MA는 원본 신호에 적용되므로 'sampling_frequency'(fs)로 계산
        body["movavg_ch"] = max(1, round(sec * fs))

    if "movavg_r_sec" in body:
        sec = body["movavg_r_sec"]
        # R MA는 시간 평균 후 신호에 적용되므로 'target_rate_hz'(tr)로 계산
        body["movavg_r"] = max(1, round(sec * tr))

    # 4. 변경된 값이 있는지 확인하고, 있다면 새로운 파라미터 사전에 업데이트
    changed = {}
    for key, value in body.items():
        if hasattr(current_params, key) and value != getattr(current_params, key):
            new_params_dict[key] = value
            changed[key] = value
    
    # 5. C 코드에 영향을 주는 파라미터 중 하나라도 바뀌면 재시작
    restarted = False
    critical_keys = ["sampling_frequency", "block_samples", "target_rate_hz", "lpf_cutoff_hz", "movavg_r", "movavg_ch"]
    if any(k in changed for k in critical_keys):
        p_current = app.state.pipeline
        p_current.stop()
        
        # 업데이트된 파라미터 사전으로 새 PipelineParams 객체 생성
        new_params_obj = PipelineParams(**new_params_dict)
        
        # 새 파이프라인 생성 및 시작
        new_pipeline = Pipeline(params=new_params_obj, broadcast_fn=p_current.broadcast_fn)
        new_pipeline.start()
        app.state.pipeline = new_pipeline # 앱의 상태를 새 파이프라인으로 교체
        restarted = True
        print("[INFO] Pipeline has been restarted due to critical parameter change.")
    
    # 6. 최종 결과 반환
    return {
        "ok": True, 
        "changed": changed, 
        "restarted": restarted,
        "params": _with_legacy_keys(asdict(app.state.pipeline.params))
    }





@app.post("/api/params/reset")
async def reset_params():
    """파라미터를 기본값으로 되돌리고 파이프라인을 재시작합니다."""
    p_current = app.state.pipeline
    p_current.stop()

    # 기본 파라미터 불러오기
    base = deepcopy(app.state.default_params)
    # 실행 관련 값은 현재 pipeline 것 유지
    base.mode = p_current.params.mode
    base.exe_path = p_current.params.exe_path
    base.ip = p_current.params.ip
    # base.block_samples = p_current.params.block_samples
    # base.sampling_frequency = p_current.params.sampling_frequency

    new_pipeline = Pipeline(params=base, broadcast_fn=p_current.broadcast_fn)
    new_pipeline.start()
    app.state.pipeline = new_pipeline

    payload = {"type": "params", "data": _with_legacy_keys(asdict(new_pipeline.params))}
    app.state.pipeline._broadcast(payload)  # 초기화된 값 즉시 push

    return {"ok": True, "restarted": True, "params": _with_legacy_keys(asdict(new_pipeline.params))}



def _send_cfg_if_serial(pipeline):
    # Serial TX 포트가 있을 때만 전송
    sp = getattr(pipeline.params, "serial", None)
    if not sp or not pipeline.source or not getattr(pipeline.source, "tx", None):
        return
    params = pipeline.params
    cfg_line = encode_cfg(
        lpf_cutoff_hz = params.lpf_cutoff_hz,
        sampling_rate = params.sampling_frequency/1000.0,  # kS/s로 맞춘다면 변환 주의
        target_rate   = params.target_rate_hz,
        movavg_r      = params.movavg_r,
        movavg_ch     = params.movavg_ch,
        channel_mask  = 255,  # 8채널 고정
        block_size    = params.block_samples,
        coeffs_y1     = params.y1_den,   # alias 주의
        coeffs_y2     = params.y2_coeffs,
        coeffs_y3     = params.y3_coeffs,
        coeffs_yt     = [params.E, params.F],
    )
    pipeline.source.tx.write_line(cfg_line)




@app.get("/favicon.ico")
async def favicon():
    return Response(status_code=204)


# -----------------------------
# [WebSocket] 실시간 데이터 전송용 엔드포인트
# -----------------------------
# • 브라우저가 /ws 로 접속하면, 서버는 연결을 수락하고
# pipeline.register_consumer()로 큐(Queue)를 하나 만들어 연결에 묶어줍니다.
# • pipeline._run()이 큐에 넣는 JSON 문자열을 이 WS가 꺼내서 클라이언트로 send_text 합니다.
# • 연결이 끊기면(WebSocketDisconnect) 조용히 종료합니다.

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    q = app.state.pipeline.register_consumer()
    # 최초 접속 시 현재 파라미터를 먼저 내려주어 UI 초기 상태를 맞춥니다.
    await ws.send_json({"type": "params", "data": _with_legacy_keys(asdict(app.state.pipeline.params))})
    try:
        while True:
            msg = await q.get()        # pipeline이 큐에 넣은 문자열(JSON)
            await ws.send_text(msg)    # 브라우저로 전송
    except WebSocketDisconnect:
        pass
    finally:
        pass


# -----------------------------
# [Entrypoint] 스크립트를 직접 실행했을 때만 동작
# -----------------------------
# • 명령줄 인자(--mode, --uri 등)를 읽어 초기 PipelineParams를 구성하고
# Pipeline을 시작한 뒤, FastAPI 서버를 uvicorn으로 띄웁니다.
# • app.state.default_params: "초기화 버튼"이 참조할 원본 파라미터 보관용
# • app.state.pipeline: 현재 실행 중인 파이프라인 보관


if __name__ == "__main__":
    # --- 1. 명령줄 인자 파싱 ---
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["synthetic", "cproc", "serial"], default="cproc") # serial 추가
    parser.add_argument("--uri", type=str, default="192.168.1.133", help="device IP")
    parser.add_argument("--fs", type=float, default=100000, help="ADC sampling frequency (Hz)")
    parser.add_argument("--block", type=int, default=16384, help="Samples per block")
    parser.add_argument("--exe", type=str, default="iio_reader.exe", help="Path to C executable")
    parser.add_argument("--host", type=str, default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    
    # serial I/O 옵션
    parser.add_argument("--rx_port", type=str, default="COM11")
    parser.add_argument("--rx_baud", type=int, default=115200)
    parser.add_argument("--tx_port", type=str, default=None)
    parser.add_argument("--tx_baud", type=int, default=115200)

    args = parser.parse_args()

    # --- 2. 서버 시작 시의 기본 파라미터 생성 ---
    # 이 객체는 '초기화' 버튼의 기준값이 됨
    startup_params = PipelineParams(
        # 명령줄 인자로 받은 실행 파라미터
        mode=args.mode,
        exe_path=args.exe,
        ip=args.uri,
        block_samples=args.block,
        sampling_frequency=args.fs,
        
        # DSP 관련 기본값은 dataclass 정의를 따름
        # 이 값들이 UI의 초기 슬라이더 위치를 결정
        target_rate_hz=10.0,
        lpf_cutoff_hz=2500.0,
        movavg_r=5,
        label_names=["yt0", "yt1", "yt2", "yt3"],
        # 나머지 계수들은 PipelineParams에 정의된 기본값을 사용
    )
    
    #serial 파라미터 주입
    if args.mode == "serial":
        # 문자열 "None", "", "null" → 실제 None 처리
        def _norm(v):
            if v is None: return None
            s = str(v).strip().lower()
            return None if s in ("none", "", "null") else v
        startup_params.serial = adc_pipeline.SerialParams(
            port=_norm(args.rx_port),
            baud=args.rx_baud,
            tx_port=_norm(args.tx_port),
            tx_baud=args.tx_baud,
        )


    # --- 3. 파이프라인 생성 및 시작 ---
    #  [수정] pipeline에 startup_params의 '복사본'을 전달하여
    # default_params가 오염되지 않도록 합니다.
    pipeline = Pipeline(params=deepcopy(startup_params), broadcast_fn=lambda payload: None)
    pipeline.start()


    # --- 4. FastAPI 앱 상태(app.state)에 객체 저장 ---
    # '초기화' 버튼이 참조할 수 있도록 원본/기본 파라미터를 저장
    app.state.default_params = startup_params
    # 현재 실행 중인 파이프라인 저장
    app.state.pipeline = pipeline


    # --- 5. 서버 실행 ---
    print(f"[INFO] pipeline loaded with params: {_with_legacy_keys(asdict(pipeline.params))}")
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")