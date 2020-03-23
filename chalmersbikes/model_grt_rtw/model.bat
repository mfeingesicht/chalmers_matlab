set MATLAB=C:\Program Files\MATLAB

cd .

if "%1"=="" ("C:\PROGRA~1\MATLAB\bin\win64\gmake"  -f model.mk all) else ("C:\PROGRA~1\MATLAB\bin\win64\gmake"  -f model.mk %1)
@if errorlevel 1 goto error_exit

exit 0

:error_exit
echo The make command returned an error of %errorlevel%
exit 1
