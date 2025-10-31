@echo off
chcp 65001 >nul

ECHO [ START ] [DSP 서버] 가상 모드(Synthetic)로 웹 서버를 시작합니다...
ECHO [ Warning ] 이 콘솔 창을 닫으면 프로그램이 종료됩니다 !!
ECHO [ 기다려주십시오 ] 잠시 후 웹 브라우저가 자동으로 열립니다.

:: 1) 서버 실행 (필요시 COM 포트/속도 변경)
START "DSP Server (RS-485)" DSP_SERVER.exe --mode serial --host 127.0.0.1 --port 8000 --rx_port COM10 --rx_baud 115200

:: 2) 서버 기동 대기
timeout /t 5 /nobreak >nul

:: 3) 브라우저 열기
start http://127.0.0.1:8000

ECHO [ ON ] 프로그램이 실행 중입니다.
pause >nul
