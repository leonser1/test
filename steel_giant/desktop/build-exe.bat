@echo off
chcp 65001 >nul
cd /d "%~dp0"
if not exist node_modules call npm install
call npm run build
echo.
echo Готово: файл SteelGiant.exe лежит в папке dist
pause
