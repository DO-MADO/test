// ============================================================
//  [Chart.js & Zoom Plugin 초기화 구간]
// ------------------------------------------------------------
//  - Chart.js: 메인 그래프 렌더링 라이브러리
//  - chartjs-plugin-zoom: 마우스 휠 / 드래그 / 핀치 확대·축소 기능 제공
//  - ESM(ECMAScript Module) 방식으로 CDN에서 직접 import
//  - 반드시 Chart.register(...) 호출 후 플러그인 활성화 필요
// ============================================================

// Chart.js 본체 + 자동 타입 감지(import from CDN, ESM)
import Chart from 'https://cdn.jsdelivr.net/npm/chart.js@4.4.1/auto/+esm';

// Zoom 플러그인 모듈 import
import zoomPlugin from 'https://cdn.jsdelivr.net/npm/chartjs-plugin-zoom@2.0.1/+esm';

// Chart.js에 zoom 플러그인 등록 (등록하지 않으면 확대/축소 기능 작동 X)
Chart.register(zoomPlugin);


// ❗ [신규 추가] 페이지 로드 시의 기본 파라미터를 저장할 변수
let initialParams = null;

// ✅ 차트에 표시할 최대 데이터 포인트 수 (메모리 관리)
const MAX_DATA_POINTS = 36000;

// ============================================================
//  [DOM 요소 참조]
// ------------------------------------------------------------
//  - HTML 내 id를 가진 주요 UI 요소를 캐싱해 변수에 보관
//  - querySelector 대신 getElementById 사용 (성능 + 명확성)
//  - 이후 이벤트 바인딩 / 데이터 반영 시 이 변수들을 직접 활용
// ============================================================

// --- Figure 1 차트 캔버스 ---
const fig1Ctx = document.getElementById('fig1'); // 원신호(멀티채널) 표시용

// --- Figure 3: 파라미터 관련 컨트롤 ---
const paramsView = document.getElementById('paramsView'); // 파라미터 요약 텍스트 영역
const lpf = document.getElementById('lpf'); // LPF 슬라이더
const lpfNum = document.getElementById('lpf_num'); // LPF 수치 입력창
const maCh = document.getElementById('ma_ch_sec'); // CH 이동평균 슬라이더
const maChNum = document.getElementById('ma_ch_sec_num'); // CH 이동평균 수치 입력창
const maR = document.getElementById('ma_r_sec'); // R 이동평균 슬라이더
const maRNum = document.getElementById('ma_r_sec_num'); // R 이동평균 수치 입력창
const tRate = document.getElementById('trate'); // Target Rate 슬라이더
const tRateNum = document.getElementById('trate_num'); // Target Rate 수치 입력창
const resetParamsBtn = document.getElementById('resetParams'); // 파라미터 초기화 버튼

// --- Figure 1: 채널 토글 바 & 줌 리셋 ---
const fig1Bar = document.getElementById('fig1Bar'); // 채널 on/off 버튼 그룹
const resetBtn1 = document.getElementById('resetZoom1'); // 줌 리셋 버튼

// --- Figure 2: 계수 입력 영역 ---
const y1c = document.getElementById('y1c'); // y1 계수 입력창
const y2c = document.getElementById('y2c'); // y2 계수 입력창
const y3c = document.getElementById('y3c'); // y3 계수 입력창
const ytc = document.getElementById('ytc'); // yt 계수 입력창
const saveY1 = document.getElementById('saveY1'); // y1 저장 버튼
const saveY2 = document.getElementById('saveY2'); // y2 저장 버튼
const saveY3 = document.getElementById('saveY3'); // y3 저장 버튼
const saveYt = document.getElementById('saveYt'); // yt 저장 버튼
// ❗ [추가] 4ch 탭의 실시간 yt 값 표시 영역
const ytValuesDisplay = document.getElementById('ytValuesDisplay');

// ❗ [신규 추가]
const maR_raw = document.getElementById('ma_r_sec_raw');
const maRNum_raw = document.getElementById('ma_r_sec_num_raw');
const applyBtn_raw_r = document.getElementById('apply_raw_r');
const resetParamsBtn_raw_r = document.getElementById('resetParams_raw_r');

// --- 색상 팔레트 (채널별 라인 색상, 최대 8채널까지 반복 적용) ---
const palette = [
  '#60A5FA',
  '#F97316',
  '#34D399',
  '#F472B6',
  '#A78BFA',
  '#EF4444',
  '#22D3EE',
  '#EAB308',
];

// --- Sampling Rate / Block Size 입력 요소 ---
const fsRate = document.getElementById('fs_rate'); // 샘플링 속도 슬라이더 (kS/s 단위)
const fsRateNum = document.getElementById('fs_rate_num'); // 샘플링 속도 수치 입력
const blockSize = document.getElementById('block_size'); // 블록 크기 슬라이더
const blockSizeNum = document.getElementById('block_size_num'); // 블록 크기 수치 입력

const statsDisplay = document.getElementById('statsDisplay');

// [추가] Raw Data 상세 보기 모드용 DOM
const rawViewBothBtn = document.getElementById('rawViewBothBtn');
const rawViewS3Btn = document.getElementById('rawViewS3Btn');
const rawViewS5Btn = document.getElementById('rawViewS5Btn');
const viewModeBtns = [rawViewBothBtn, rawViewS3Btn, rawViewS5Btn]; // 버튼 그룹

const stage3Block = document.getElementById('stage3Block');
const stage5Block = document.getElementById('stage5Block');
const stage3Container = document.getElementById('stage3Container');
const stage5Container = document.getElementById('stage5Container');


// 전역 초기화 버튼
// ❗ [수정]
function resetAllChartData() {
  // 1. 모든 차트의 데이터를 초기화합니다.
  resetFig1Data();
  resetFig3Data();
  // ❗ [수정] 예전 함수(resetYtChartData) 대신 새 함수(resetYtChannelStages)를 호출합니다.
  ytChannels.forEach(ch => resetYtChannelStages(ch));

  // 2. 사용자에게 알림창을 띄웁니다.
  alert("모든 그래프가 초기화 되었습니다.");
}



// ============================================================
//  [4ch(fig2) 탭 리팩토링: 관련 DOM 참조 및 상태 관리]
// ============================================================
const fig2Refs = {
  charts: {},       // 예: charts['yt0']['y2'] = Chart instance
  contexts: {},
  timeCounters: {},
  wrappers: {},
  // 버튼들은 이제 사용되지 않으므로 제거하거나 주석처리 가능
  // dataResetButtons: {},
  // zoomResetButtons: {},
};

const ytChannels = ['yt0', 'yt1', 'yt2', 'yt3'];
const ytStages = ['y2', 'y3', 'yt']; // ❗ [추가] 스테이지 목록



// ❗ [수정] 4개 채널 x 3개 스테이지에 대한 DOM 요소를 반복문으로 찾아와 fig2Refs 객체에 저장
ytChannels.forEach((ch) => {
  // 채널별로 객체 초기화
  fig2Refs.charts[ch] = {};
  fig2Refs.contexts[ch] = {};
  fig2Refs.timeCounters[ch] = {};

  fig2Refs.wrappers[ch] = document.getElementById(`${ch}Wrapper`);

  ytStages.forEach((stage) => {
    const canvasId = `fig2_${ch}_${stage}`;
    const canvas = document.getElementById(canvasId);
    if (canvas) {
      fig2Refs.contexts[ch][stage] = canvas;
      fig2Refs.timeCounters[ch][stage] = 0;
    }
  });
});

const fig2GridContainer = document.getElementById('fig2GridContainer');
const ytViewModeBar = document.getElementById('ytViewModeBar');



// --- 보기 모드 적용 함수 ---
function applyRawViewMode(mode) {
  if (mode === 'stage3') {
    stage3Block.style.display = '';
    stage5Block.style.display = 'none';
    stage3Container.classList.add('large');
    stage5Container.classList.remove('large');
  } else if (mode === 'stage5') {
    stage3Block.style.display = 'none';
    stage5Block.style.display = '';
    stage5Container.classList.add('large');
    stage3Container.classList.remove('large');
  } else {
    // 'both' 모드
    stage3Block.style.display = '';
    stage5Block.style.display = '';
    stage3Container.classList.remove('large');
    stage5Container.classList.remove('large');
  }

  // Chart.js에 리사이즈/업데이트 알리기
  setTimeout(() => {
    fig1?.resize();
    fig3?.resize();
    fig1?.update('none');
    fig3?.update('none');
  }, 0);
}

// --- 'active' 클래스 관리 헬퍼 함수 ---
function updateActiveButton(activeBtn) {
  viewModeBtns.forEach((btn) => {
    btn?.classList.toggle('active', btn === activeBtn);
  });
}

// --- 새로운 버튼 클릭 이벤트 리스너 설정 함수 ---
function setupViewModeButtons() {
  rawViewBothBtn?.addEventListener('click', () => {
    applyRawViewMode('both');
    updateActiveButton(rawViewBothBtn);
  });
  rawViewS3Btn?.addEventListener('click', () => {
    applyRawViewMode('stage3');
    updateActiveButton(rawViewS3Btn);
  });
  rawViewS5Btn?.addEventListener('click', () => {
    applyRawViewMode('stage5');
    updateActiveButton(rawViewS5Btn);
  });
}

// ============================================================
//  [슬라이더와 숫자 입력 상호 동기화]
// ============================================================

/**
 * ✅ Y축 범위 설정 UI의 이벤트 리스너를 설정하는 함수
 */
function setupYAxisControls() {
  // '범위 적용' 버튼 (기존 로직)
  yApply1.addEventListener('click', () => {
    const min = parseFloat(yMin1.value);
    const max = parseFloat(yMax1.value);
    if (!isNaN(min)) fig1.options.scales.y.min = min;
    if (!isNaN(max)) fig1.options.scales.y.max = max;

    fig1.update();
  });

  // --- fig1 Y축 자동 ---
  yAuto1.addEventListener('click', () => {
    fig1.options.scales.y.min = undefined;
    fig1.options.scales.y.max = undefined;
    yMin1.value = '';
    yMax1.value = '';
    fig1.update();
  });

  // --- fig3에 대해서도 동일하게 적용 ---

  // '범위 적용' 버튼 (기존 로직)
  yApply3.addEventListener('click', () => {
    const min = parseFloat(yMin3.value);
    const max = parseFloat(yMax3.value);
    if (!isNaN(min)) fig3.options.scales.y.min = min;
    if (!isNaN(max)) fig3.options.scales.y.max = max;

    // ✅ X축은 절대 건드리지 않음
    fig3.update();
  });

  // --- fig3 Y축 자동 ---
  yAuto3.addEventListener('click', () => {
    fig3.options.scales.y.min = undefined;
    fig3.options.scales.y.max = undefined;
    yMin3.value = '';
    yMax3.value = '';
    fig3.update();
  });
}

// ✅ 각 그래프 전용 시간 카운터
let lastTimeX1 = 0;
let lastTimeX3 = 0;

// ✅ Figure1 전용 리셋
function resetFig1Data() {
  lastTimeX1 = 0;
  fig1.data.labels = [];
  fig1.data.datasets.forEach((ds) => (ds.data = []));
  // 눈금(1초) 유지 + 범위 오염 방지
  fig1.options.scales.x.min = 0;
  fig1.options.scales.x.max = undefined;
  fig1.resetZoom?.();
  fig1.update('none');
}

// ✅ Figure3 전용 리셋
function resetFig3Data() {
  lastTimeX3 = 0;
  fig3.data.labels = [];
  fig3.data.datasets.forEach((ds) => (ds.data = []));
  fig3.options.scales.x.min = 0;
  fig3.options.scales.x.max = undefined;
  fig3.resetZoom?.();
  fig3.update('none');
}

// ❗ [수정] 특정 채널의 '모든 스테이지' 차트 데이터를 리셋하는 함수
function resetYtChannelStages(channel) {
  if (!fig2Refs.charts[channel]) return;

  ytStages.forEach(stage => {
    const chart = fig2Refs.charts[channel][stage];
    if (!chart) return;

    fig2Refs.timeCounters[channel][stage] = 0;
    chart.data.labels = [];
    chart.data.datasets.forEach((ds) => (ds.data = []));
    chart.options.scales.x.min = 0;
    chart.options.scales.x.max = undefined;
    chart.resetZoom?.();
    chart.update('none');
  });
}


// --- soft reconnect: 페이지 리로드 없이 C 프로세스 재시작 반영 ---
function softReconnectCharts() {
  // 1) 화면 유지 + 차트 데이터만 초기화
  resetFig1Data();
  ytChannels.forEach((ch) => resetYtChannelStages(ch));  // resetYtChartData(ch) -> resetYtChannelStages(ch)로 수정)
  resetFig3Data();

  // 2) 웹소켓만 재연결
  try {
    ws?.close();
  } catch {}
  setTimeout(connectWS, 150); // 약간의 텀을 두면 깔끔
}

// ❗ [수정] 확장된 데이터 페이로드를 모든 차트에 분배하는 함수
function appendDataToFig2(data, dt) {
  const stageDataMap = {
    y2: data.stage7_y2,
    y3: data.stage8_y3,
    yt: data.derived
  };

  ytChannels.forEach((ch, chIndex) => {
    ytStages.forEach(stage => {
      const chart = fig2Refs.charts[ch][stage];
      const sourceData = stageDataMap[stage];

      if (!chart || !sourceData || !Array.isArray(sourceData.series) || sourceData.series.length <= chIndex) {
        return;
      }
      
      const channelData = sourceData.series[chIndex];
      if (!channelData || channelData.length === 0) return;

      if (chart.data.datasets.length === 0) {
        chart.data.datasets.push({
          label: `${ch}-${stage}`,
          data: [],
          borderColor: palette[chIndex % palette.length],
          borderWidth: 1.5,
          fill: false,
          tension: 0.1,
        });
      }

      let lastTime = fig2Refs.timeCounters[ch][stage];
      const new_times = Array.from({ length: channelData.length }, (_, i) => lastTime + (i + 1) * dt);
      fig2Refs.timeCounters[ch][stage] = new_times[new_times.length - 1];

      chart.data.labels.push(...new_times);
      chart.data.datasets[0].data.push(...channelData);

      while (chart.data.labels.length > MAX_DATA_POINTS) {
        chart.data.labels.shift();
        chart.data.datasets[0].data.shift();
      }
      
      chart.update('none');
    });
  });
}



// 버튼에 연결
function setupDataResetButtons() {
  dataReset1.addEventListener('click', resetFig1Data);
  dataReset3.addEventListener('click', resetFig3Data);
}

function bindPair(rangeEl, numEl) {
  if (!rangeEl || !numEl) return; // DOM 요소가 없으면 무시
  rangeEl.addEventListener('input', () => {
    numEl.value = rangeEl.value;
  });
  numEl.addEventListener('input', () => {
    rangeEl.value = numEl.value;
  });
}

// 슬라이더 ↔ 숫자 입력창 동기화 설정
bindPair(lpf, lpfNum); // LPF Cutoff
bindPair(maCh, maChNum); // 채널 이동평균
bindPair(maR, maRNum); // R 이동평균
bindPair(tRate, tRateNum); // Target Rate
bindPair(fsRate, fsRateNum); // 샘플링 속도
bindPair(blockSize, blockSizeNum); // 블록 크기
bindPair(maR_raw, maRNum_raw);

/**
 * 차트에 데이터셋(라인)이 없으면 생성해주는 함수
 */
function ensureDatasets(chart, nCh, labelPrefix = 'ch', toggleRenderer) {
  if (chart.data.datasets.length === nCh) return;

  chart.data.datasets = Array.from({ length: nCh }, (_, k) => ({
    label: `${labelPrefix}${k}`,
    data: [], // ← XY 포인트 배열로 사용 예정
    borderColor: palette[k % palette.length],
    borderWidth: 1.5,
    fill: false,
    tension: 0.1,
    // XY 모드에서 기본 파싱 사용 (x,y 키 읽음)
    parsing: true,
    spanGaps: false,
  }));

  if (toggleRenderer) toggleRenderer(chart);
}
/**
 * 차트에 새 데이터 블록을 누적하는 함수 (수정된 버전)
 */
function appendDataToChart(chart, x_block, y_block_2d) {
  if (!y_block_2d || y_block_2d.length === 0 || y_block_2d[0].length === 0)
    return;

  const nCh = y_block_2d[0].length;
  let labelPrefix = 'ch',
    toggleRenderer = renderChannelToggles;

  if (chart.canvas.id === 'fig3') {
    labelPrefix = 'Ravg';
    toggleRenderer = renderFig3Toggles;
  }

  // 1. 데이터셋(라인)이 존재하는지 확인 및 생성
  ensureDatasets(chart, nCh, labelPrefix, toggleRenderer);

  // 2. 새 데이터를 각 데이터셋에 추가
  chart.data.labels.push(...x_block);
  for (let ch = 0; ch < nCh; ch++) {
    const newChannelData = y_block_2d.map((row) => row[ch]);
    chart.data.datasets[ch].data.push(...newChannelData);
  }

  // 3. 최대 데이터 포인트 수를 초과하면 오래된 데이터 제거
  while (chart.data.labels.length > MAX_DATA_POINTS) {
    chart.data.labels.shift();
    chart.data.datasets.forEach((dataset) => dataset.data.shift());
  }

  // 4. 차트 업데이트
  chart.update('none');
}

// ============================================================
//  [Figure 1: 채널 토글 바]
// ============================================================

let chToggleRenderedCount = 0;

function renderChannelToggles(chart) {
  const nCh = chart.data.datasets.length;
  if (
    !fig1Bar ||
    (chToggleRenderedCount === nCh && fig1Bar.childElementCount === nCh)
  )
    return;

  fig1Bar.innerHTML = '';

  for (let k = 0; k < nCh; k++) {
    const ds = chart.data.datasets[k];
    const btn = document.createElement('button');
    btn.className = 'ch-toggle';
    btn.type = 'button';
    const sw = document.createElement('span');
    sw.className = 'swatch';
    sw.style.background = ds.borderColor || '#60a5fa';
    const label = document.createElement('span');
    label.textContent = `ch${k}`;
    btn.appendChild(sw);
    btn.appendChild(label);
    if (ds.hidden) btn.classList.add('off');
    btn.addEventListener('click', () => {
      ds.hidden = !ds.hidden;
      btn.classList.toggle('off', !!ds.hidden);
      chart.update('none');
    });
    fig1Bar.appendChild(btn);
  }
  chToggleRenderedCount = nCh;
}

// ============================================================
//  [차트 생성 함수 공통화]
// ============================================================

function makeChart(
  ctx,
  { legend = false, xTitle = '', yTitle = '', decimation = true } = {}
) {
  return new Chart(ctx, {
    type: 'line',
    data: { labels: [], datasets: [] },
    options: {
      animation: false,
      responsive: true,
      maintainAspectRatio: false,
      devicePixelRatio: window.devicePixelRatio,
      interaction: { mode: 'nearest', intersect: false },
      elements: { point: { radius: 0 } },
      layout: {
        backgroundColor: '#1e1f23',
      },
      scales: {
        x: {
          type: 'linear',
          ticks: {
            stepSize: 1,
            color: '#f7f7f8',
          },
          grid: { color: '#3a3b45' },
          title: {
            display: !!xTitle,
            text: xTitle,
            color: '#f7f7f8',
          },
        },
        y: {
          ticks: { color: '#f7f7f8' },
          grid: { color: '#3a3b45' },
          title: {
            display: !!yTitle,
            text: yTitle,
            color: '#f7f7f8',
            font: { size: 14 },
          },
        },
      },
      plugins: {
        legend: {
          display: legend,
          labels: { color: '#f7f7f8' },
        },
        decimation: { enabled: decimation, algorithm: 'min-max' },
        zoom: {
          zoom: {
            wheel: { enabled: true, modifierKey: 'ctrl' },
            pinch: { enabled: true },
            drag: { enabled: true, modifierKey: 'shift' },
            mode: 'x',
          },
          pan: {
            enabled: true,
            mode: 'x',
          },
        },
        tooltip: {
          enabled: true,
          intersect: false,
          titleColor: '#f7f7f8',
          bodyColor: '#f7f7f8',
          backgroundColor: '#1e1f23',
        },
      },
    },
  });
}

// --- 실제 차트 인스턴스 생성 ---
const fig1 = makeChart(fig1Ctx, {
  xTitle: 'Time (s)',
  yTitle: 'Signal Value (V)',
});
const fig3Ctx = document.getElementById('fig3');
const resetBtn3 = document.getElementById('resetZoom3');
const fig3Bar = document.getElementById('fig3Bar');
const yMin1 = document.getElementById('yMin1'),
  yMax1 = document.getElementById('yMax1'),
  yApply1 = document.getElementById('yApply1');
const yMin3 = document.getElementById('yMin3'),
  yMax3 = document.getElementById('yMax3'),
  yApply3 = document.getElementById('yApply3');
const dataReset1 = document.getElementById('dataReset1');
const dataReset3 = document.getElementById('dataReset3');
const yAuto1 = document.getElementById('yAuto1');
const yAuto3 = document.getElementById('yAuto3');

const fig3 = makeChart(fig3Ctx, {
  xTitle: 'Time (s)',
  yTitle: 'Stage5 Ravg (unit)',
});

// --- 줌 리셋 이벤트 ---
fig1Ctx.addEventListener('dblclick', () => fig1.resetZoom());
resetBtn1?.addEventListener('click', () => fig1.resetZoom());
fig3Ctx.addEventListener('dblclick', () => fig3.resetZoom());
resetBtn3?.addEventListener('click', () => fig3.resetZoom());

// ============================================================
//  [Figure 1-2: 데이터 처리 (5단계 까지 진행한 raw 데이터)]
// ============================================================
let fig3Vis = {};
let fig3ToggleKey = '';

function renderFig3Toggles(chart) {
  if (!fig3Bar) return;
  const key = (chart.data.datasets || []).map((ds) => ds.label || '').join('|');
  if (fig3ToggleKey === key) return;
  fig3ToggleKey = key;
  fig3Bar.innerHTML = '';

  chart.data.datasets.forEach((ds, idx) => {
    const btn = document.createElement('button');
    btn.className = 'ch-toggle';
    const sw = document.createElement('span');
    sw.className = 'swatch';
    sw.style.background = ds.borderColor || '#60a5fa';
    const label = document.createElement('span');
    label.textContent = ds.label || `Ravg${idx}`;
    btn.appendChild(sw);
    btn.appendChild(label);
    if (ds.hidden) btn.classList.add('off');

    btn.addEventListener('click', () => {
      ds.hidden = !ds.hidden;
      const name = ds.label || `Ravg${idx}`;
      fig3Vis[name] = !!ds.hidden;
      btn.classList.toggle('off', !!ds.hidden);
      chart.update('none');
    });
    fig3Bar.appendChild(btn);
  });
}

// ============================================================
//  [파라미터 Fetch / 적용 / 저장]
// ============================================================
async function fetchParams() {
  const r = await fetch('/api/params');
  const p = await r.json();

  // ❗ [수정] initialParams가 비어있을 때 (최초 1회)만 값을 저장합니다.
  if (!initialParams) {
    initialParams = p;
  }

  applyParamsToUI(p);
}

function _num(v, def) {
  return typeof v === 'number' && !Number.isNaN(v) ? v : def;
}

function trimZeros(s) {
  return String(parseFloat(s));
}

function applyParamsToUI(p) {
  const fs = p.sampling_frequency ?? 100000;
  const bs = p.block_samples ?? 16384;
  const tr = p.target_rate_hz ?? 10.0;
  const lpf_hz = p.lpf_cutoff_hz ?? 2500.0;
  const ma_r_sec =
    p.movavg_r && tr > 0 ? trimZeros((p.movavg_r / tr).toFixed(6)) : '0';
  const ma_ch_sec =
    p.movavg_ch && fs > 0 ? trimZeros((p.movavg_ch / fs).toFixed(6)) : '0';

  if (fsRateNum) fsRateNum.value = (fs / 1000).toFixed(0);
  if (fsRate) fsRate.value = (fs / 1000).toFixed(0);
  if (blockSizeNum) blockSizeNum.value = bs;
  if (blockSize) blockSize.value = bs;
  if (tRateNum) tRateNum.value = tr;
  if (tRate) tRate.value = tr;
  if (lpfNum) lpfNum.value = lpf_hz;
  if (lpf) lpf.value = lpf_hz;
  if (maRNum) maRNum.value = ma_r_sec;
  if (maR) maR.value = ma_r_sec;
  if (maChNum) maChNum.value = ma_ch_sec;
  if (maCh) maCh.value = ma_ch_sec;
  if (maRNum_raw) maRNum_raw.value = ma_r_sec; // ❗ [신규 추가]
  if (maR_raw) maR_raw.value = ma_r_sec;     // ❗ [신규 추가]

  

  // ❗ [추가] 4ch 탭 계수 입력 필드 채우기
  if (y1c && p.y1_den) y1c.value = p.y1_den.join(',');
  if (y2c && p.y2_coeffs) y2c.value = p.y2_coeffs.join(',');
  if (y3c && p.y3_coeffs) y3c.value = p.y3_coeffs.join(',');
  if (ytc && p.E !== undefined && p.F !== undefined) ytc.value = `${p.E},${p.F}`;
  



  if (paramsView) {
    const fsPretty =
      fs >= 1e6 ? `${(fs / 1e6).toFixed(2)} MS/s` : `${(fs / 1e3).toFixed(0)} kS/s`;
    const blockSec = fs > 0 ? trimZeros((bs / fs).toFixed(6)) : '0';
    const idealBlocks = fs > 0 && bs > 0 ? fs / bs : 0;
    const chStep = fs > 0 ? trimZeros((1 / fs).toFixed(6)) : '0';
    const ma_ch_samples = Number.isFinite(p.movavg_ch) ? p.movavg_ch : 0;
    const ma_r_samples = Number.isFinite(p.movavg_r) ? p.movavg_r : 0;
    const y1_den = p.y1_den || p.coeffs_y1 || [];
    const y2_cs = p.y2_coeffs || p.coeffs_y2 || [];
    const y3_cs = p.y3_coeffs || p.coeffs_y3 || [];
    const yt_cs =
      p.coeffs_yt || (p.E != null && p.F != null ? [p.E, p.F] : []);
    const HILITE = (v) =>
      `<span style="color: rgb(96, 165, 250); font-weight: 550">${v}</span>`;
    paramsView.innerHTML = `
      <br/>
      <h1>🖥️ 통합 상태 요약</h1>
      <p><strong>샘플링 속도(ADC)</strong> : ${HILITE(fsPretty)} <span class="hint">— 하드웨어가 초당 채취하는 원시 샘플 개수</span></p>
      <p><strong>블록 크기</strong> : ${HILITE(`${bs} 샘플`)} <span class="hint">— 블록 1개의 길이 약 ${HILITE(blockSec + '초')}(ADC 기준)</span></p>
      <p><strong>표출 속도(시간평균 후)</strong> : ${HILITE(`${tr} 샘플/초`)} <span class="hint">— ADC 샘플을 평균 내어 초당 ${tr}개 점으로 줄여 화면에 표시</span></p>
      <p><strong>LPF 차단 주파수</strong> : ${HILITE(`${lpf_hz} Hz`)} <span class="hint">— 노이즈 억제를 위한 저역통과 필터 설정</span></p>
      <p><strong>채널 이동평균(ADC 도메인)</strong> : ${HILITE(`${ma_ch_sec}s`)} <span class="hint">— 원시 신호에서 주변 샘플을 평균하여 매끄럽게 표시 (창 크기: ${HILITE(`${ma_ch_samples}샘플`)}, 최소 해상도: ${HILITE(`${chStep}s`)})</span></p>
      <p><strong>R 이동평균(시간평균 도메인)</strong> : ${HILITE(`${ma_r_sec}s`)} <span class="hint">— 로그비(R) 계산 결과를 추가로 평활화 (창 크기: ${HILITE(`${ma_r_samples}샘플`)} @ 표출 속도 ${HILITE(`${tr}샘플/초`)})</span></p>
      <br/>
      <p><strong>파생 지표(계산치)</strong></p>
      <p>· <strong>이상적 블록 취득 시간</strong> <em>(T_block = block_samples / sampling_frequency)</em> : ${HILITE(`${blockSec} s`)}</p>
      <p>· <strong>이상적 블록 처리율</strong> <em>(sampling_frequency / block_samples)</em> : ${HILITE(`${trimZeros(idealBlocks.toFixed(2))} 블록/초`)}</p>
      <br/>
      ${Array.isArray(y1_den) && y1_den.length ? `<p><strong>y1 분모 계수</strong> : ${HILITE('[' + y1_den.join(' , ') + ']')}</p>` : ''}
      ${Array.isArray(y2_cs) && y2_cs.length ? `<p><strong>y2 보정 계수</strong> : ${HILITE('[' + y2_cs.join(' , ') + ']')}</p>` : ''}
      ${Array.isArray(y3_cs) && y3_cs.length ? `<p><strong>y3 보정 계수</strong> : ${HILITE('[' + y3_cs.join(' , ') + ']')}</p>` : ''}
      ${Array.isArray(yt_cs) && yt_cs.length ? `<p><strong>yt 스케일 계수(E, F)</strong> : ${HILITE('[' + yt_cs.join(' , ') + ']')}</p>` : ''}
    `;
  }
}

async function postParams(diff) {
  if (diff.movavg_r_sec !== undefined) {
    const sec = parseFloat(diff.movavg_r_sec) || 0;
    const tr = parseFloat(tRateNum.value) || 10;
    diff.movavg_r = Math.max(1, Math.round(sec * tr));
  }
  const r = await fetch('/api/params', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(diff),
  });
  const j = await r.json();
  if (j.restarted) {
    softReconnectCharts();
  } else {
    applyParamsToUI(j.params);
  }
}


// ❗ [추가] alert 창에 표시될 계수 이름을 매핑합니다.
const coeffDisplayNames = {
    'y1_den': 'y1 분모',
    'y2_coeffs': 'y2',
    'y3_coeffs': 'y3',
    'yt_coeffs': 'yt'
};

async function postCoeffs(key, values) {
  try {
    const r = await fetch('/api/coeffs', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ key, values }),
    });
    if (!r.ok) {
        throw new Error(`Server responded with ${r.status}`);
    }

    // ❗ --- [여기가 추가된 부분입니다] ---
    // 1. 매핑된 이름 가져오기 (없으면 원래 key 사용)
    const displayName = coeffDisplayNames[key] || key;
    // 2. 사용자에게 보여줄 alert 메시지 생성 및 표시
    // JSON.stringify(values)는 [0,1,0.5] 와 같이 배열 형태로 예쁘게 만들어줍니다.
    alert(`${displayName} 계수 ${JSON.stringify(values)} (이)가 적용되었습니다.`);
    // ❗ --- [여기까지] ---

    const j = await r.json();
    // 성공 시, Configuration 탭의 파라미터 뷰를 최신 정보로 업데이트
    if (j.ok && j.params) {
        applyParamsToUI(j.params);
    }
    console.log(j.message);
  } catch (e) {
    console.error(`Failed to update coeffs for ${key}:`, e);
    alert(`'${key}' 계수 업데이트에 실패했습니다.`);
  }
}



document.getElementById('apply')?.addEventListener('click', async () => { // ❗ 1. async 추가
  try {
    // ❗ 2. await를 사용하여 postParams 함수가 완료될 때까지 기다림
    await postParams({
      sampling_frequency: parseFloat(fsRateNum.value) * 1000,
      block_samples: parseInt(blockSizeNum.value, 10),
      target_rate_hz: parseFloat(tRateNum.value),
      lpf_cutoff_hz: parseFloat(lpfNum.value),
      movavg_r_sec: parseFloat(maRNum.value),
      movavg_ch_sec: parseFloat(maChNum.value),
    });

    // ❗ 3. 성공적으로 완료되면 alert 알림창 표시
    alert("파라미터가 성공적으로 적용되었습니다.");

  } catch (e) {
    // (선택) 만약 서버 요청 중 에러가 발생하면 사용자에게 알림
    console.error("파라미터 적용 실패:", e);
    alert("파라미터 적용 중 오류가 발생했습니다.");
  }
});

// ❗ [신규 추가] Raw Data 탭의 R Moving Avg '적용' 버튼
applyBtn_raw_r?.addEventListener('click', () => {
  // R Moving Avg 파라미터 하나만 객체에 담아 전송
  postParams({
    movavg_r_sec: parseFloat(maRNum_raw.value),
  });
});

// ❗ [수정] Raw Data 탭의 R Moving Avg '적용' 버튼 이벤트 리스너
apply_raw_r?.addEventListener('click', async () => { // ❗ async 키워드 추가
  try {
    // 1. 적용할 값을 변수에 저장
    const valueToApply = parseFloat(maRNum_raw.value);

    // 2. await를 사용해 서버에 적용이 끝날 때까지 기다림
    await postParams({
      movavg_r_sec: valueToApply,
    });

    // 3. 성공적으로 끝나면 사용자에게 알림창 표시
    alert(`R Moving Avg 값이 ${valueToApply}s 로 적용되었습니다.`);

  } catch (e) {
    // 4. 중간에 에러가 발생하면 콘솔에 기록
    console.error('Failed to apply R Moving Avg:', e);
    alert('값 적용에 실패했습니다.');
  }
});

// ❗ [신규 추가] Raw Data 탭의 '초기화' 버튼
resetParamsBtn_raw_r?.addEventListener('click', () => {
  // 1. 저장해둔 초기 파라미터가 없으면 아무것도 하지 않음
  if (!initialParams) {
    alert("초기 파라미터 정보가 아직 로드되지 않았습니다.");
    return;
  }

  // 2. 초기 파라미터에서 R Moving Avg의 기본값(초 단위)을 계산
  const default_movavg_r = initialParams.movavg_r;
  const default_tr = initialParams.target_rate_hz;
  const defaultValueSec = (default_movavg_r / default_tr).toFixed(1);

  // 3. 계산된 기본값으로 postParams 함수를 호출하여 서버에 적용
  //    UI는 postParams가 성공 응답을 받으면 자동으로 업데이트됩니다.
  postParams({ movavg_r_sec: parseFloat(defaultValueSec) });

  // 4. 사용자에게 피드백
  alert(`R Moving Avg 값이 초기값(${defaultValueSec}s)으로 재설정되었습니다.`);
});


function parseCoeffs(txt) {
  return txt
    .split(',')
    .map((s) => parseFloat(s.trim()))
    .filter((v) => !Number.isNaN(v));
}

function parseCoeffsN(txt, n = 6, fill = 0) {
  const v = parseCoeffs(txt);
  while (v.length < n) v.push(fill);
  if (v.length > n) v.length = n;
  return v;
}

function _info(msg) {
  alert(msg);
}



// ❗ [수정] 기존 _info 알림창을 postCoeffs 호출로 변경
saveY1?.addEventListener('click', () => {
    const values = parseCoeffs(y1c.value);
    postCoeffs('y1_den', values);
});
saveY2?.addEventListener('click', () => {
    const values = parseCoeffs(y2c.value);
    postCoeffs('y2_coeffs', values);
});
saveY3?.addEventListener('click', () => {
    const values = parseCoeffs(y3c.value);
    postCoeffs('y3_coeffs', values);
});
saveYt?.addEventListener('click', () => {
    const values = parseCoeffs(ytc.value);
    postCoeffs('yt_coeffs', values);
});



function updateStatsDisplay(stats) {
  if (!statsDisplay || !stats) return;

  const fs = stats.sampling_frequency;
  const fsDisplay =
    fs >= 1_000_000
      ? `${(fs / 1_000_000).toFixed(1)} MS/s`
      : `${(fs / 1000).toFixed(1)} kS/s`;

  const items = [
    `설정 샘플링: <span class="stat-value">${fsDisplay}</span>`,
    `설정 블록 크기: <span class="stat-value">${stats.block_samples} samples</span>`,
    `블록 처리 시간: <span class="stat-value">${stats.actual_block_time_ms.toFixed(
      2
    )} ms</span>`,
    `초당 처리 블록: <span class="stat-value">${stats.actual_blocks_per_sec.toFixed(
      2
    )} blocks/s</span>`,
    `최종 출력 속도: <span class="stat-value">${(
      stats.actual_proc_Sps || 0
    ).toFixed(2)} S/s/ch</span>`,
  ];

  statsDisplay.innerHTML = items.join('<span class="separator"> | </span>');
}

/**
 * ❗ [신규 추가] 4ch 탭의 실시간 yt 값 텍스트를 업데이트하는 함수
 * @param {Array<number|null>} latestValues - [yt0, yt1, yt2, yt3] 최신 값 배열
 */
function updateYtValuesDisplay(latestValues) {
  // DOM 요소가 없거나, 배열 데이터가 아니면 즉시 종료
  if (!ytValuesDisplay || !Array.isArray(latestValues)) return;

  // 배열의 각 값을 "yt0: 1.2345" 형태의 HTML 문자열로 변환
  const items = latestValues.map((v, i) => {
    // 값이 null이거나 유효하지 않으면 '---'로 표시, 아니면 소수점 4자리까지
    const valueStr = v === null || v === undefined ? '---' : v.toFixed(4);
    return `yt${i}: <span class="stat-value">${valueStr}</span>`;
  });

  // 변환된 문자열들을 구분자와 함께 합쳐서 innerHTML에 삽입
  ytValuesDisplay.innerHTML = items.join('<span class="separator"> | </span>');
}


// ============================================================
//  데이터 저장 위한 수집
// ============================================================
// ❗ [추가] 모든 차트에서 데이터를 수집하는 함수
function gatherAllChartData() {
  const allData = {
    stage3: {
      labels: fig1.data.labels,
      datasets: fig1.data.datasets.map(ds => ({ label: ds.label, data: ds.data }))
    },
    stage5: {
      labels: fig3.data.labels,
      datasets: fig3.data.datasets.map(ds => ({ label: ds.label, data: ds.data }))
    },
    stages789: {}
  };

  ytChannels.forEach(ch => {
    allData.stages789[ch] = {};
    ytStages.forEach(stage => {
      const chart = fig2Refs.charts[ch][stage];
      if (chart) {
        allData.stages789[ch][stage] = {
          labels: chart.data.labels,
          datasets: chart.data.datasets.map(ds => ({ label: ds.label, data: ds.data }))
        };
      }
    });
  });

  return allData;
}



// ============================================================
//  [WebSocket 연결 & 데이터 핸들링]
// ============================================================
let ws;

function connectWS() {
  const url =
    (location.protocol === 'https:' ? 'wss://' : 'ws://') +
    location.host +
    '/ws';
  ws = new WebSocket(url);

  ws.onopen = () => {};

  ws.onmessage = (ev) => {
  try {
    const m = JSON.parse(ev.data);

    if (m.type === 'params') {
      applyParamsToUI(m.data);
      return;
    }

    if (m.type === 'frame') {
      const tRate = Number(m?.params?.target_rate_hz);
      const dt = tRate > 0 ? 1.0 / tRate : null;
      const y_block = Array.isArray(m.y_block) ? m.y_block : null;
      const ravg_block =
        m.ravg_signals && Array.isArray(m.ravg_signals.series)
          ? m.ravg_signals.series
          : null;

      // --- Raw Data (Stage3) 그래프 데이터 처리 ---
      if (y_block && y_block.length > 0 && dt !== null) {
        const n1 = y_block.length;
        const new_times1 = Array.from(
          { length: n1 },
          (_, i) => lastTimeX1 + (i + 1) * dt
        );
        lastTimeX1 = new_times1[new_times1.length - 1];
        appendDataToChart(fig1, new_times1, y_block);
      }

      // --- Raw Data (Stage5) 그래프 데이터 처리 ---
      if (ravg_block && ravg_block.length > 0 && dt !== null) {
        const chCount = ravg_block.length;
        const sampleCount = Array.isArray(ravg_block[0])
          ? ravg_block[0].length
          : 0;
        if (sampleCount > 0) {
          const ravg_transposed = Array.from(
            { length: sampleCount },
            (_, s) =>
              Array.from({ length: chCount }, (_, c) => ravg_block[c][s])
          );
          const n3 = sampleCount;
          const new_times3 = Array.from(
            { length: n3 },
            (_, i) => lastTimeX3 + (i + 1) * dt
          );
          lastTimeX3 = new_times3[new_times3.length - 1];
          appendDataToChart(fig3, new_times3, ravg_transposed);
        }
      }
      
      // --- 4ch 탭 (Stage 7, 8, 9) 그래프 데이터 처리 ---
      if (m.derived && m.stage7_y2 && m.stage8_y3 && dt !== null) {
        appendDataToFig2(m, dt);
      }
      
      // --- 4ch 탭 실시간 텍스트 값 표시 (최종 YT 값만 사용) ---
      if (m.derived && m.derived.series) {
        const latestYtValues = m.derived.series.map((channelData) =>
          channelData.length > 0 ? channelData[channelData.length - 1] : null
        );
        updateYtValuesDisplay(latestYtValues);
      }

      // --- 헤더 통계 정보 업데이트 ---
      if (m.stats) {
        updateStatsDisplay(m.stats);
      }
      return;
    }
  } catch (e) {
    console.error('WebSocket message parse error', e);
  }
};

  ws.onerror = (e) => {
    console.error('[WS] error', e);
  };

  ws.onclose = () => {
    setTimeout(connectWS, 1000);
  };
}

// ============================================================
//  [파라미터 초기화 버튼]
// ============================================================
resetParamsBtn?.addEventListener('click', async () => {
  try {
    const r = await fetch('/api/params/reset', { method: 'POST' });
    const j = await r.json();
    
    if (j.ok) {
      if (j.params) {
        applyParamsToUI(j.params);
      }
      
      
      if (j.restarted) {
        softReconnectCharts();
      }
      
      alert("모든 파라미터 값과 계수가 초기화 되었습니다.");

    } else {
      alert("초기화에 실패했습니다.");
    }
  } catch (e) {
    console.error(e);
    alert("초기화 중 오류가 발생했습니다.");
  }
});



// ❗ [수정] 4ch 탭 뷰 모드 전환 로직
function setupYtViewMode() {
  const buttons = ytViewModeBar.querySelectorAll('.view-mode-btn');

  buttons.forEach((btn) => {
    btn.addEventListener('click', () => {
      // --- 스타일링 로직 ---
      buttons.forEach((b) => {
        b.classList.remove('active');
        // ❗ [추가] 이전에 적용된 인라인 배경색 스타일을 모두 초기화합니다.
        b.style.backgroundColor = '';
      });
      btn.classList.add('active');
      
      const target = btn.dataset.ytTarget;
      const channelIndex = ytChannels.indexOf(target); // yt0=0, yt1=1, ...

      // ❗ [추가] yt1, yt2, yt3일 때만 동적으로 배경색을 변경합니다.
      // channelIndex가 1 이상일 경우 (yt1, yt2, yt3)
      if (channelIndex >= 1) {
        // palette 배열에서 해당 채널의 색상을 가져와 버튼 배경색으로 직접 설정
        btn.style.backgroundColor = palette[channelIndex];
      }
      
      // --- 레이아웃 변경 로직 (기존과 동일) ---
      if (target === 'all') {
        fig2GridContainer.classList.add('all-view');
        fig2GridContainer.classList.remove('single-view');
        ytChannels.forEach((ch) => {
          fig2Refs.wrappers[ch].classList.remove('visible');
        });
      } else {
        fig2GridContainer.classList.add('single-view');
        fig2GridContainer.classList.remove('all-view');
        ytChannels.forEach((ch) => {
          fig2Refs.wrappers[ch].classList.toggle('visible', ch === target);
        });
      }

      // --- 차트 리사이즈 로직 (기존과 동일) ---
      setTimeout(() => {
        Object.values(fig2Refs.charts).forEach((channelCharts) => {
          Object.values(channelCharts).forEach(chart => chart.resize());
        });
      }, 0);
    });
  });
   
  document.querySelector('.view-mode-btn[data-yt-target="all"]').click();
}


// ============================================================
//  [초기 실행]
// ============================================================

// 4개 채널 x 3개 스테이지 차트 인스턴스 생성 및 이벤트 리스너 설정
ytChannels.forEach((ch) => {
  ytStages.forEach(stage => {
    const chartCtx = fig2Refs.contexts[ch][stage];
    if (!chartCtx) return;

    // 차트 생성
    fig2Refs.charts[ch][stage] = makeChart(chartCtx, {
      xTitle: 'Time (s)',
      yTitle: `${ch} ${stage} (unit)`,
      decimation: false, // 상세 뷰이므로 데시메이션 비활성화
    });
    
    // 이벤트 리스너 연결을 위한 DOM 요소 ID 가져오기
    const dataResetBtn = document.getElementById(`dataReset2_${ch}_${stage}`);
    const zoomResetBtn = document.getElementById(`resetZoom2_${ch}_${stage}`);
    
    // 이벤트 리스너 할당
    chartCtx.addEventListener('dblclick', () => fig2Refs.charts[ch][stage].resetZoom());
    zoomResetBtn?.addEventListener('click', () => fig2Refs.charts[ch][stage].resetZoom());
    dataResetBtn?.addEventListener('click', () => {
      // 개별 차트 데이터 리셋 로직
      const chart = fig2Refs.charts[ch][stage];
      if (!chart) return;

      fig2Refs.timeCounters[ch][stage] = 0;
      chart.data.labels = [];
      chart.data.datasets.forEach((ds) => (ds.data = []));
      chart.options.scales.x.min = 0;
      chart.options.scales.x.max = undefined;
      chart.resetZoom?.();
      chart.update('none');
    });
  });
});

// --- 필수 함수들 호출 ---
connectWS();
fetchParams();
setupYAxisControls();
setupDataResetButtons();
applyRawViewMode('both');
setupViewModeButtons();
setupYtViewMode(); // 4ch 탭 뷰 모드 활성화

// 'Reset' 탭 버튼 이벤트 리스너
const globalResetBtn = document.getElementById('globalResetBtn');
if (globalResetBtn) {
  globalResetBtn.addEventListener('click', resetAllChartData);
}



// ============================================================
//  [파일 저장]
// ============================================================
// ❗ [추가] Save 버튼 이벤트 리스너
const saveDataBtn = document.getElementById('saveDataBtn');
if (saveDataBtn) {
  saveDataBtn.addEventListener('click', async () => {
    // 1. 사용자에게 저장 여부 확인
    if (!confirm("지금까지의 누적 데이터를 CSV 파일로 저장하시겠습니까?")) {
      return; // '아니오' 선택 시 아무것도 하지 않음
    }

    try {
      // 2. 모든 차트에서 데이터 수집
      const chartData = gatherAllChartData();

      // 3. 백엔드 API로 데이터 전송
      const response = await fetch('/api/save_data', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(chartData),
      });

      const result = await response.json();

      // 4. 결과에 따라 사용자에게 피드백
      if (result.ok) {
        alert(`데이터가 성공적으로 저장되었습니다.\n경로: ${result.message}`);
      } else {
        throw new Error(result.message);
      }
    } catch (e) {
      console.error("데이터 저장 실패:", e);
      alert(`데이터 저장 중 오류가 발생했습니다: ${e.message}`);
    }
  });
}