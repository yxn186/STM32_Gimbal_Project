@echo off
echo =============^> GeneratorBefore.bat run ^<=============

set "main_c_file=%~dp0Core\Src\main.c"
set "main_cpp_file=%~dp0Core\Src\main.cpp"

echo main_c_file  = "%main_c_file%"
echo main_cpp_file= "%main_cpp_file%"

if exist "%main_cpp_file%" (
    REM If a stale main.c exists, remove it to avoid rename failure
    if exist "%main_c_file%" (
        echo Found existing main.c, deleting it...
        del /f /q "%main_c_file%"
        if errorlevel 1 (
            echo [ERR] Failed to delete main.c
            exit /b 11
        )
        echo Deleted main.c OK
    )

    echo Found main.cpp
    ren "%main_cpp_file%" main.c
    if errorlevel 1 (
        echo [ERR] Rename failed: main.cpp to main.c
        exit /b 12
    )
    echo Rename OK: main.cpp to main.c
) else (
    echo [ERR] main.cpp not found
    exit /b 10
)

echo =============^> GeneratorBefore.bat stop ^<=============
