@echo off
SetLocal EnableExtensions
set file=..\Core\Inc\version.h

For /F "UseBackQ tokens=1-3 delims==( " %%a in ("%file%") do if "%%b"=="MAJOR_VERSION" set maj_ver=%%c
echo Version1=%maj_ver%
For /F "UseBackQ tokens=1-3 delims==( " %%a in ("%file%") do if "%%b"=="MINOR_VERSION" set min_ver=%%c
echo Version1=%min_ver%
For /F "UseBackQ tokens=1-3 delims==( " %%a in ("%file%") do if "%%b"=="REVISION_VERSION" set rev_ver=%%c
echo Version1=%rev_ver%

COPY /Y bkte_2_0_iar1\Exe\bkte_2_0_iar1.bin bkte_2_0_iar1\Exe\121_bkte_noboot_%maj_ver%.%min_ver%.%rev_ver%_%date:~-10,2%%date:~-7,2%%date:~-4,4%.bin
