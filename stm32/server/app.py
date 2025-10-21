#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import json
import importlib.util
from dataclasses import asdict 
from pathlib import Path
import sys
import pandas as pd
from typing import Optional, List

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Response
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn
from pydantic import BaseModel, Field
from typing import Optional, List, Dict #  [추가] Dict 임포트
from copy import deepcopy
from datetime import datetime
from zoneinfo import ZoneInfo


# -----------------------------
# Paths
# -----------------------------
ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
PIPELINE_PATH = ROOT / "pipeline.py"
COEFFS_JSON = ROOT / "coeffs.json"

# -----------------------------
# Import pipeline.py dynamically
# -----------------------------
spec = importlib.util.spec_from_file_location("adc_pipeline", str(PIPELINE_PATH))
adc_pipeline = importlib.util.module_from_spec(spec)
sys.modules["adc_pipeline"] = adc_pipeline
assert spec.loader is not None
spec.loader.exec_module(adc_pipeline)

Pipeline = adc_pipeline.Pipeline
PipelineParams = adc_pipeline.PipelineParams

# -----------------------------
# Pydantic 모델
# -----------------------------

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
# FastAPI app & Helpers
# -----------------------------
app = FastAPI(title="AD4858 Realtime Web UI")
if STATIC.exists():
    app.mount("/static", StaticFiles(directory=str(STATIC)), name="static")

def _with_legacy_keys(p: dict) -> dict:
    if "y1_den" in p: p["coeffs_y1"] = p["y1_den"]
    if "y2_coeffs" in p: p["coeffs_y2"] = p["y2_coeffs"]
    if "y3_coeffs" in p: p["coeffs_y3"] = p["y3_coeffs"]
    if "E" in p and "F" in p: p["coeffs_yt"] = [p["E"], p["F"]]
    return p

# -----------------------------
# Routes
# -----------------------------
@app.get("/")
async def index():
    return FileResponse(STATIC / "index.html")


#  [추가] 데이터 처리 및 CSV 저장을 위한 헬퍼 함수
def process_and_save_csv(all_data: AllChartData, file_path: Path, start_ts: float):
    """
    모든 차트 데이터를 병합하여 단일 CSV 파일로 저장합니다. (리샘플링 없음)
    """
    all_series = []
    
    #  [수정] start_ts 인자를 받도록 시그니처 변경
    def create_series_from_chart_data(chart_data: ChartData, base_name: str, start_ts: float) -> list:
        series_list = []
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




#  [추가] 데이터 저장을 위한 새로운 API 엔드포인트
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




#  [신규 추가] 계수 업데이트를 위한 API 엔드포인트
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



@app.get("/favicon.ico")
async def favicon():
    return Response(status_code=204)


# -----------------------------
# WebSocket
# -----------------------------
@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    q = app.state.pipeline.register_consumer()
    await ws.send_json({"type": "params", "data": _with_legacy_keys(asdict(app.state.pipeline.params))})
    try:
        while True:
            msg = await q.get()
            await ws.send_text(msg)
    except WebSocketDisconnect:
        pass
    finally:
        pass

# -----------------------------
# Entrypoint (최종 수정 버전)
# -----------------------------
if __name__ == "__main__":
    # --- 1. 명령줄 인자 파싱 ---
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["synthetic", "cproc"], default="cproc")
    parser.add_argument("--uri", type=str, default="192.168.1.133", help="device IP")
    parser.add_argument("--fs", type=float, default=100000, help="ADC sampling frequency (Hz)")
    parser.add_argument("--block", type=int, default=16384, help="Samples per block")
    parser.add_argument("--exe", type=str, default="iio_reader.exe", help="Path to C executable")
    parser.add_argument("--host", type=str, default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
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