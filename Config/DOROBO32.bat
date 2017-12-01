@ECHO OFF
REM CLS
REM COM PORT Nummer in Geraetemanager nachschauen und eintragen
SET COMPORT=3

REM Manuelle Uebergabe des Dateinamens
REM SET HEXFILEPATH="C:\Users\Public\Documents\Dorobo32\Installation\Programming Software\DFU\dfu util 0.8\DOROBO32_CubemxTEST.hex"

REM Eclipse Uebergabe des Dateinamens mit "${workspace_loc:/${project_name}}/${config_name:/${project_name}}/${project_name}.hex"
REM ECHO Argument 1 ist %1
SET HEXFILEPATH=%1

ECHO Flashing DOROBO32 on COM%COMPORT%
ECHO Flashing file %HEXFILEPATH%
ECHO (This may take some 10 seconds)

"C:\Program Files (x86)\STMicroelectronics\Software\Flash Loader Demo\STMFlashLoader.exe" -c --pn %COMPORT% -Dtr --Hi -Rts --Hi
"C:\Program Files (x86)\STMicroelectronics\Software\Flash Loader Demo\STMFlashLoader.exe" -c --pn %COMPORT% --br 256000 --db 8 --pr EVEN --sb 1 --ec OFF  --to 500 --co Off -i STM32F0_7x_128K -e --all -d --v --o --fn %HEXFILEPATH% -r --a 8000000
"C:\Program Files (x86)\STMicroelectronics\Software\Flash Loader Demo\STMFlashLoader.exe" -c --pn %COMPORT% -Dtr --Hi
REM PAUSE