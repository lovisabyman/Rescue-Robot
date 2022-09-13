for /f %%i in ('where.exe gui.py') do set guipath=%%i
echo gui path ar %guipath%
SETLOCAL ENABLEDELAYEDEXPANSION
SET count=1
FOR /F "tokens=* USEBACKQ" %%F IN (`where.exe python.exe`) DO (
  SET pypath!count!=%%F
  SET /a count=!count!+1
)
"%pypath1%" "%guipath%"
ENDLOCAL
pause
