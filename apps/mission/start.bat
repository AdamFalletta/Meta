@echo off
cd /d "%~dp0"
echo Starting AFI Mission Control...
echo Open http://localhost:8080 in your browser.
node serve.js
pause
