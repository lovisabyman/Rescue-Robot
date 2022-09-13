SETLOCAL ENABLEDELAYEDEXPANSION
SET count=1
FOR /F "tokens=* USEBACKQ" %%F IN (`where.exe python.exe`) DO (
  SET pypath!count!=%%F
  SET /a count=!count!+1
)
"%pypath1%" "%~dp0%\websocket\socket_server.py"
ENDLOCAL
pause
