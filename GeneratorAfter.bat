@echo off
echo ============= GeneratorAfter.bat run =============

set "main_c_file=%~dp0Core\Src\main.c"
set "main_cpp_file=%~dp0Core\Src\main.cpp"

if exist "%main_c_file%" (
    if exist "%main_cpp_file%" (
        echo [ERR] main.cpp already exists, abort.
        exit /b 20
    )
    ren "%main_c_file%" main.cpp
    if errorlevel 1 (
        echo [ERR] Rename failed.
        exit /b 21
    )
    echo OK renamed main.c to main.cpp
) else (
    echo [ERR] main.c not found
    exit /b 30
)

set "usb_c_file=%~dp0USB_DEVICE\App\usbd_cdc_if.c"
if exist "%usb_c_file%" (
    del /f /q "%usb_c_file%"
    if errorlevel 1 (
        echo [ERR] Delete usbd_cdc_if.c failed.
        exit /b 31
    )
    echo OK deleted usbd_cdc_if.c
) else (
    echo [WARN] usbd_cdc_if.c not found, skip delete
)

echo ============= GeneratorAfter.bat stop =============
