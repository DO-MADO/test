@echo off
ECHO [AD4858] 웹 서버를 시작합니다...
ECHO 이 검은색 콘솔 창을 닫으면 프로그램이 종료됩니다.
ECHO 잠시 후 웹 브라우저가 자동으로 열립니다.

:: 1. 우리가 만든 .exe 서버를 "백그라운드"로 실행합니다.
:: (파일 이름이 AD4858_Synthetic.exe가 맞는지 확인하세요)
START "AD4858 Server" AD4858_Synthetic.exe

:: 2. 서버가 켜질 때까지 2초 정도 기다립니다.
:: (PC가 느리면 3이나 5로 늘려주세요)
timeout /t 2 /nobreak > nul

:: 3. PC의 기본 웹 브라우저로 로컬호스트 주소를 엽니다.
start http://127.0.0.1:8000

ECHO 프로그램이 실행 중입니다.