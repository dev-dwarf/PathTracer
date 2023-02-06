@echo off

IF NOT EXIST build mkdir build

pushd build

set LIBS=-I C:\Code\
set DISABLED= -wd4201 -wd4100 -wd4189 -wd4244 -wd4456 -wd4457 -wd4245
set FLAGS= -nologo -FC -GR- -Oi -Zi -W4 %DISABLED% /TP

rem del *.pdb > NUL 2> NUL

cl %FLAGS% %LIBS% ..\src\main.cpp || exit /b

main.exe || exit /b

cover.png

popd

