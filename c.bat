@echo off
chcp 65001 > nul
SET FILENAME="Debug\kit_kirokukai2021_gr_peach.bin"

if exist "d:\MBED.HTM" (
  echo D Drive Copying...
  copy %FILENAME% d:
  goto EXIT
) else (
  echo D Drive not Drive.
)

if exist "e:\MBED.HTM" (
  echo E Drive Copying...
  copy %FILENAME% e:
  goto EXIT
) else (
  echo E Drive not Drive.
)

if exist "f:\MBED.HTM" (
  echo F Drive Copying...
  copy %FILENAME% f:
  goto EXIT
) else (
  echo F Drive not Drive.
)

if exist "g:\MBED.HTM" (
  echo G Drive Copying...
  copy %FILENAME% g:
  goto EXIT
) else (
  echo G Drive not Drive.
)

if exist "h:\MBED.HTM" (
  copy %FILENAME% h:
  goto EXIT
) else (
  echo H Drive not Drive.
)

if exist "I:\MBED.HTM" (
  copy %FILENAME% i:
  goto EXIT
) else (
  echo I Drive not Drive.
)

if exist "j:\MBED.HTM" (
  copy %FILENAME% j:
  goto EXIT
) else (
  echo J Drive not Drive.
)

if exist "k:\MBED.HTM" (
  copy %FILENAME% k:
  goto EXIT
) else (
  echo K Drive not Drive.
)

if exist "l:\MBED.HTM" (
  copy %FILENAME% l:
  goto EXIT
) else (
  echo L Drive not Drive.
)

if exist "m:\MBED.HTM" (
  copy %FILENAME% m:
  goto EXIT
) else (
  echo M Drive not Drive.
)

if exist "n:\MBED.HTM" (
  copy %FILENAME% n:
  goto EXIT
) else (
  echo N Drive not Drive.
)

if exist "o:\MBED.HTM" (
  copy %FILENAME% o:
  goto EXIT
) else (
  echo O Drive not Drive.
)

if exist "p:\MBED.HTM" (
  copy %FILENAME% p:
  goto EXIT
) else (
  echo P Drive not Drive.
)

if exist "q:\MBED.HTM" (
  copy %FILENAME% q:
  goto EXIT
) else (
  echo Q Drive not Drive.
)

if exist "r:\MBED.HTM" (
  copy %FILENAME% r:
  goto EXIT
) else (
  echo R Drive not Drive.
)

if exist "s:\MBED.HTM" (
  copy %FILENAME% s:
  goto EXIT
) else (
  echo S Drive not Drive.
)

if exist "t:\MBED.HTM" (
  copy %FILENAME% t:
  goto EXIT
) else (
  echo T Drive not Drive.
)

if exist "u:\MBED.HTM" (
  copy %FILENAME% u:
  goto EXIT
) else (
  echo U Drive not Drive.
)

if exist "v:\MBED.HTM" (
  copy %FILENAME% v:
  goto EXIT
) else (
  echo V Drive not Drive.
)

if exist "w:\MBED.HTM" (
  copy %FILENAME% w:
  goto EXIT
) else (
  echo W Drive not Drive.
)

if exist "x:\MBED.HTM" (
  copy %FILENAME% x:
  goto EXIT
) else (
  echo X Drive not Drive.
)

if exist "y:\MBED.HTM" (
  copy %FILENAME% y:
  goto EXIT
) else (
  echo Y Drive not Drive.
)

if exist "z:\MBED.HTM" (
  copy %FILENAME% z:
  goto EXIT
) else (
  echo Z Drive not Drive.
)

Echo GR-PEACH Drive Not Found.
goto EXIT_ERROR

:EXIT
Echo =====================================
Echo ========== Copy finished!! ==========
Echo =====================================
ping 127.0.0.1 -n 3 > nul
:EXIT_ERROR
