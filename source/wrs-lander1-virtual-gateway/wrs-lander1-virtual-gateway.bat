@ECHO off

REM
REM Copyright (c) 2015 Wind River Systems, Inc.
REM
REM The right to copy, distribute, modify or otherwise make use
REM of this software may be licensed only pursuant to the terms
REM of an applicable Wind River license agreement.
REM
REM

SETLOCAL EnableExtensions EnableDelayedExpansion

SET DEVICE_DIR=%~dp0
SET DEVICE_FILE=virtual-gateway\appvirtgw.exe

SET deviceName=wrs-lander1
SET serverURL=https://app.cloud.windriver.com/
SET deviceId=wrs-lander1_661d807a-9493-4f18-a574-b45b77dddbe0
SET deviceManagerWS=WSS:52.24.248.3:443;GetUrl=/devmgr/v1//wrs-lander1_661d807a-9493-4f18-a574-b45b77dddbe0;ID=wrs-lander1_Gateway_f64bf4ed-192b-4ff9-b066-411d75c41135
SET serialParams=115200-8-N-1-N
SET version=0.2

SET Path=%SystemRoot%\System32\;%SystemRoot%;%SystemRoot%\System32\Wbem;%Path%

TITLE "Helix App Cloud Virtual Gateway for %deviceName%"

ECHO(
ECHO Helix App Cloud Virtual Gateway for device %deviceName% on server %serverURL%
ECHO Version: %version%
ECHO(

SET serial_baudrate_default=115200
SET serial_port_default=NothingChosen
SET serial_default_file="%DEVICE_DIR%\serial_port.txt"
SET proxy_default_file="%DEVICE_DIR%\proxy.txt"
SET host_proxy_default=NothingChosen
SET port_proxy_default=NothingChosen

SET serial_port=NothingChosen
SET serial_baudrate=NothingChosen
SET proxy_host=NothingChosen
SET proxy_port=NothingChosen

IF EXIST %serial_default_file% (
    FOR /F "tokens=1,2" %%I IN ('type %serial_default_file%') DO (
        SET serial_port_default=%%I
        SET serial_baudrate_default=%%J
    )
)
IF EXIST %proxy_default_file% (
    FOR /F "tokens=1,2" %%I IN ('type %proxy_default_file%') DO (
        SET host_proxy_default=%%I
        SET port_proxy_default=%%J
    )
) else (
    FOR /f "tokens=3" %%a IN ('reg query "HKCU\Software\Microsoft\Windows\CurrentVersion\Internet Settings" /v ProxyEnable 2^>NUL ^| find /i "0x1"') do (
        FOR /f "tokens=3" %%a IN ('reg query "HKCU\Software\Microsoft\Windows\CurrentVersion\Internet Settings" /v ProxyServer 2^>NUL') do (
	    SET proxy_list=%%a
        )
    )
)

:_parseproxy_list
IF ["%proxy_list%"] == [""] GOTO _donegetproxy
FOR /F "tokens=1* delims=;" %%A IN ("%proxy_list%") DO (
    echo %%A | findstr /B "socks=" 1>NUL 2>NUL && SET sock_proxy=%%A
    echo %%A | findstr /C:"=" 1>NUL 2>NUL || SET global_proxy=%%A
    SET proxy_list=%%B
)
GOTO _parseproxy_list
:_donegetproxy

if ["%global_proxy%"] == [""] GOTO _getsockproxy
FOR /F "tokens=1,2 delims=:" %%A in ("%global_proxy%") DO (
    SET host_proxy_system=%%A
    SET port_proxy_system=%%B
)
SET host_proxy_default=%host_proxy_system%
SET port_proxy_default=%port_proxy_system%

:_getsockproxy
if ["%sock_proxy%"] == [""] GOTO _getserialports
SET sock_proxy=%sock_proxy:~6%
FOR /F "tokens=1,2 delims=:" %%A in ("%sock_proxy%") DO (
    SET host_proxy_system=%%A
    SET port_proxy_system=%%B
)
SET host_proxy_default=%host_proxy_system%
SET port_proxy_default=%port_proxy_system%

:_getserialports
for /f "tokens=3" %%a in ('reg query hklm\hardware\devicemap\serialcomm ^| find /i "COM"') do (
    @ECHO OFF
    if defined COMPORTS (SET COMPORTS=!COMPORTS!, %%a) else (SET COMPORTS=%%a)
)

ECHO Provide the name of the serial port your device is connected to:

:_getserial
SET INVALID_SERIAL=0
IF ["%serial_port_default%"] == ["NothingChosen"] (
    SET /P serial_port="Serial port: " || SET serial_port=NothingChosen
) ELSE (
    SET /P serial_port="Serial port (default is %serial_port_default%): " || SET serial_port=%serial_port_default%
)

IF ["%serial_port%"] == ["NothingChosen"] GOTO _getserial
IF X%serial_port:COM=%==X%serial_port% (
    ECHO Error: Invalid serial port name %serial_port%
    ECHO Valid serial port names are: %COMPORTS%
    ECHO:
    SET INVALID_SERIAL=1
)
IF %INVALID_SERIAL% EQU 1 GOTO _getserial

SET FOUND_SERIAL=0
FOR %%a in (%COMPORTS%) do (
    IF %%a EQU %serial_port%  (
	SET FOUND_SERIAL=1
    )
)


IF %FOUND_SERIAL% NEQ 1 (
    ECHO Error: "%serial_port%" is not a valid serial port.
    ECHO Valid serial port names are: %COMPORTS%
    SET INVALID_SERIAL=1
)

IF %INVALID_SERIAL% EQU 1 GOTO _getserial

IF ["%serialParams%"] NEQ [""] GOTO _haveSerialParams

for %%a in (110 300 600 1200 2400 4800 9600 14400 19200 28800 38400 56000 57600 115200 230400 460800 921600) do SET %%a=1

:_getbaudrate
SET INVALID_BAUDRATE=0

SET /P serial_baudrate="Serial port baudrate (default is %serial_baudrate_default%): " || SET serial_baudrate=NothingChosen
IF ["%serial_baudrate%"] == ["NothingChosen"] (
    SET serial_baudrate=%serial_baudrate_default%
)
IF NOT DEFINED %serial_baudrate% (
    ECHO Error: unsupported serial line baudrate.
    SET serial_baudrate=NothingChosen
    SET INVALID_BAUDRATE=1
)

IF %INVALID_BAUDRATE% EQU 1 GOTO _getbaudrate

SET serialParams=%serial_baudrate%

GOTO _storeParams

:_haveSerialParams
ECHO Connecting serial port %serial_port% with serial parameters (from the SDK): %serialParams%
ECHO:

:_storeParams

REM Store the serial name and baudrate localy
ECHO %serial_port% %serial_baudrate% > %serial_default_file%

IF ["%host_proxy_default%"] == ["NothingChosen"] (
    SET default=n
) ELSE (
    SET default=y
)

:_getproxy
SET ENABLE_PROXY=NothingChosen
SET /P KEY="Configure a SOCKS v5 proxy to access App Cloud server (y/n, default is %default%): " || SET KEY=NothingChosen
IF ["%KEY%"] == ["N"] SET ENABLE_PROXY=n
IF ["%KEY%"] == ["n"] SET ENABLE_PROXY=n
IF ["%KEY%"] == ["Y"] SET ENABLE_PROXY=y
IF ["%KEY%"] == ["y"] SET ENABLE_PROXY=y
IF ["%KEY%"] == ["NothingChosen"] SET ENABLE_PROXY=%default%
if ["%ENABLE_PROXY%"] == ["NothingChosen"] GOTO _getproxy

SET PROXY_ARG=
if ["%ENABLE_PROXY%"] == ["n"] (
    @DEL /S %proxy_default_file% > NUL 2> NUL
    GOTO _start
)

:_getproxyhost
IF ["%host_proxy_default%"] == ["NothingChosen"] (
    SET /P host_proxy="SOCKS v5 proxy host: " || SET host_proxy=NothingChosen
) ELSE (
    SET /P host_proxy="SOCKS v5 proxy host (default is %host_proxy_default%): " || SET host_proxy=%host_proxy_default%
)
IF ["%host_proxy%"] == ["NothingChosen"] GOTO _getproxyhost

:_getproxyport
IF ["%port_proxy_default%"] == ["NothingChosen"] (
    SET /P port_proxy="SOCKS v5 proxy port: " || SET port_proxy=NothingChosen
) ELSE (
    SET /P port_proxy="SOCKS v5 proxy port (default is %port_proxy_default%): " || SET port_proxy=%port_proxy_default%
)
IF ["%port_proxy%"] == ["NothingChosen"] GOTO _getproxyport

SET proxy_args=-p %host_proxy%:%port_proxy%
REM Store the proxy configuration locally
SET save_proxy_config=0
if NOT ["%host_proxy%"] == ["%host_proxy_default%"] SET save_proxy_config=1
if NOT ["%port_proxy%"] == ["%port_proxy_default%"] SET save_proxy_config=1
IF %save_proxy_config% EQU 1 ECHO %host_proxy% %port_proxy% > %proxy_default_file%

:_start
ECHO(
ECHO Starting the Virtual Gateway for %deviceName%.
START /W /B "Helix App Cloud Virtual Gateway for %deviceName%" /D "%DEVICE_DIR%" "%DEVICE_DIR%\%DEVICE_FILE%" %proxy_args% -device serial:%serial_port%,Params=%serialParams%,ID=%deviceId% %deviceManagerWS%
