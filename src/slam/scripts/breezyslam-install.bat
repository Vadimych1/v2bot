@echo off
setlocal enabledelayedexpansion

if "%~1"=="" (
    set "PYTHON=python"
) else (
    set "PYTHON=%~1"
)

cd breezyslam
rmdir /S /Q dist >nul 2>&1
rmdir /S /Q build >nul 2>&1
%PYTHON% -m build --wheel

set "target_folder=dist"

for /f "delims=" %%i in ('dir /b /a-d "%target_folder%\*" 2^>nul') do (
    set "first_file=%%i"
    goto :found
)

echo build not found
exit /b

:found
set "file_path=%target_folder%\!first_file!"

%PYTHON% -m pip install "%file_path%" --force-reinstall