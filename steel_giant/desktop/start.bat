@echo off
chcp 65001 >nul
cd /d "%~dp0"
where npm >nul 2>nul || (echo Нужен Node.js LTS: https://nodejs.org & pause & exit /b 1)
if not exist node_modules\electron\dist\electron.exe (
  echo Первый запуск: скачиваю Electron, это 1-2 минуты...
  call npm install || (pause & exit /b 1)
)
start "" "node_modules\electron\dist\electron.exe" .
