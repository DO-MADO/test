@echo off
ECHO [AD4858] STM32 연결 모드(Serial)로 웹 서버를 시작합니다...
ECHO 사용할 COM 포트: RX=COM17, TX=COM13 (필요시 이 파일을 수정하세요)
ECHO 이 검은색 콘솔 창을 닫으면 프로그램이 종료됩니다.
ECHO 잠시 후 웹 브라우저가 자동으로 열립니다.

:: 1. 우리가 만든 .exe 서버를 "백그라운드"로 실행 + 필요한 옵션 전달!
:: (파일 이름이 AD4858_Server.exe가 맞는지 확인)
START "AD4858 Server" AD4858_Server.exe --mode serial --host 127.0.0.1 --port 8000 --rx_port COM17 --rx_baud 115200 --tx_port COM13 --tx_baud 115200

:: 2. 서버가 켜질 때까지 2초 정도 기다립니다.
timeout /t 2 /nobreak > nul

:: 3. PC의 기본 웹 브라우저로 로컬호스트 주소를 엽니다.
start http://127.0.0.1:8000

ECHO 프로그램이 실행 중입니다.