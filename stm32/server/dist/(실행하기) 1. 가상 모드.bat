@echo off
ECHO [DSP 서버] 가상 모드(Synthetic)로 웹 서버를 시작합니다...
ECHO 이 검은색 콘솔 창을 닫으면 프로그램이 종료됩니다. [cite: 5]
ECHO 잠시 후 웹 브라우저가 자동으로 열립니다. [cite: 5]

:: 1. DSP_SERVER.EXE 서버를 "백그라운드"로 실행합니다.
:: (파일 이름이 DSP_SERVER.EXE가 맞는지 확인하세요)
START "DSP Server" DSP_SERVER.EXE --mode synthetic --host 127.0.0.1 --port 8000

:: 2. 서버가 켜질 때까지 n초 정도 기다립니다. [cite: 7]
:: (PC가 느리면 3이나 5로 늘려주세요) [cite: 7]
timeout /t 5 /nobreak > nul

:: 3. PC의 기본 웹 브라우저로 로컬호스트 주소를 엽니다.
start http://127.0.0.1:8000 [cite: 8]

ECHO 프로그램이 실행 중입니다.