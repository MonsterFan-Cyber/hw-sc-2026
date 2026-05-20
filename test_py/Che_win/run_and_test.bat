@echo off
setlocal DISABLEDELAYEDEXPANSION

:: ==============================================
:: Script Started
:: ==============================================
echo ==============================================
echo Judgement Script Started
echo ==============================================

set "SOLUTION_FILE=%~1"
set "INPUT_FILE=%~2"
set "OUTPUT_FILE=%~3"
set "TIMEOUT_MS=%~4"

if "%SOLUTION_FILE%"=="" (
    echo.
    echo Usage: %0 ^<solution_file^> ^<input_file^> ^<output_file^> [timeout_ms]
    echo Example: %0 .\demos\Solution.cpp .\data\sample.in .\out.txt 90000
    echo Supported: C, CPP, JAVA, PYTHON
    echo.
    exit /b 5
)
if "%INPUT_FILE%"=="" (
    echo [ERROR] Input file not specified
    exit /b 5
)
if "%OUTPUT_FILE%"=="" (
    echo [ERROR] Output file not specified
    exit /b 5
)

echo [INFO] Solution: %SOLUTION_FILE%
echo [INFO] Input:    %INPUT_FILE%
echo [INFO] Output:   %OUTPUT_FILE%
echo [INFO] Timeout:  %TIMEOUT_MS%ms
echo.

set "LANG_TYPE="
for %%A in ("%SOLUTION_FILE%") do set "EXT=%%~xA"

if /i "%EXT%"==".c"    set LANG_TYPE=C
if /i "%EXT%"==".cpp"  set LANG_TYPE=CPP
if /i "%EXT%"==".java" set LANG_TYPE=JAVA
if /i "%EXT%"==".py"   set LANG_TYPE=PYTHON

if "%LANG_TYPE%"=="" (
    echo [ERROR] Unsupported file type: %EXT%
    exit /b 5
)
echo [INFO] Language: %LANG_TYPE%
echo.

for %%f in ("%SOLUTION_FILE%") do (
    set "BASE_NAME=%%~nf"
    set "SOLUTION_PATH=%%~dpf"
)

if "%SOLUTION_PATH:~-1%"=="\" set "SOLUTION_PATH=%SOLUTION_PATH:~0,-1%"

echo [INFO] Solution Path: %SOLUTION_PATH%
echo [INFO] Base Name:     %BASE_NAME%
echo.

:: Check Runner.exe
if not exist "Runner.exe" (
    echo [ERROR] Runner.exe NOT FOUND!
    exit /b 1
)
echo [INFO] Runner.exe Check Passed
echo.

:: ================= Compile =================
if /i "%LANG_TYPE%"=="C"     goto compile_c
if /i "%LANG_TYPE%"=="CPP"   goto compile_cpp
if /i "%LANG_TYPE%"=="JAVA"  goto compile_java
if /i "%LANG_TYPE%"=="PYTHON" goto compile_python

echo [ERROR] Unsupported Language
exit /b 5

:compile_c
echo [INFO] Compiling C Program...
gcc -std=c17 "%SOLUTION_FILE%" -o "%SOLUTION_PATH%\%BASE_NAME%.exe" -O2 -lm
if errorlevel 1 (
    echo [ERROR] C Compile Failed
    exit /b 2
)
if not exist "%SOLUTION_PATH%\%BASE_NAME%.exe" (
    echo [ERROR] Exe File Not Found
    exit /b 6
)
echo [INFO] C Compile Success
goto run_runner

:compile_cpp
echo [INFO] Compiling C++ Program...
g++ -std=c++17 "%SOLUTION_FILE%" -o "%SOLUTION_PATH%\%BASE_NAME%.exe" -O2
if errorlevel 1 (
    echo [ERROR] C++ Compile Failed
    exit /b 2
)
if not exist "%SOLUTION_PATH%\%BASE_NAME%.exe" (
    echo [ERROR] Exe File Not Found
    exit /b 6
)
echo [INFO] C++ Compile Success
goto run_runner

:compile_java
echo [INFO] Compiling Java Program...
javac -d "%SOLUTION_PATH%" -implicit:none "%SOLUTION_FILE%"
if errorlevel 1 (
    echo [ERROR] Java Compile Failed
    exit /b 2
)
if not exist "%SOLUTION_PATH%\%BASE_NAME%.class" (
    echo [ERROR] Class File Not Found
    exit /b 6
)
echo [INFO] Java Compile Success
goto run_runner

:compile_python
echo [INFO] Python No Compile Needed
goto run_runner

:: ================= Run =================
:run_runner
echo.
echo ==============================================
echo Running Program...
echo ==============================================
Runner.exe "%INPUT_FILE%" "%OUTPUT_FILE%" "%LANG_TYPE%" "%SOLUTION_PATH%" "%BASE_NAME%" "%TIMEOUT_MS%"

echo.
echo [INFO] Process Finished
echo ==============================================
echo.

endlocal
exit /b 0