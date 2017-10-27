CLS
@ECHO OFF
ECHO DFU Flash for DOROBO32
ECHO Dorobo32: BOOT druecken, RESET druecken, RESET loslassen, BOOT loslassen
ECHO Dorobo32 in Programmierstimmung  ;-)
 
REM Nur der Pfad zur BIN-Datei muss angepasst werden:
"C:\Users\Public\Documents\Dorobo32\Installation\Programming Software\DFU\dfu util 0.8\dfu-util.exe" -a 0 -s 0x8000000:leave -D "C:\Users\Public\Documents\Dorobo32\Installation\Programming Software\DFU\dfu util 0.8\DOROBO32_CubemxTEST_DFU.bin" -d 0483:df11
PAUSE 