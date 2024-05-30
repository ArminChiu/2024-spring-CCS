@echo off

setlocal enableextensions enabledelayedexpansion
goto begin

:runTestcase 
    set testcase=%~1
    (echo ^`define TRACE_FILE ^"test/sim/%testcase%.mem.log^"
    echo ^`define INST_MEM ^"test/sim/%testcase%.mem^"
    ) > test/define.v
    echo --------------------- %testcase% ---------------------
    iverilog -o code_run ./test/testbench.v ./submit/code.v
    vvp code_run 
GOTO :EOF

:runMediumTestcase 
    call :runTestcase sum
    call :runTestcase mov-c
    call :runTestcase fib
    call :runTestcase add
    call :runTestcase if-else
    call :runTestcase pascal
    call :runTestcase quick-sort
    call :runTestcase select-sort
    call :runTestcase max
    call :runTestcase min
    call :runTestcase switch
    call :runTestcase bubble-sort
GOTO :EOF

:runAdvancedTestcase 
    call :runTestcase sub-longlong
    call :runTestcase bit
    call :runTestcase fact
    call :runTestcase add-longlong
    call :runTestcase "shift"
    call :runTestcase wanshu
    call :runTestcase goldbach
    call :runTestcase leap-year
    call :runTestcase prime
    call :runTestcase mul-longlong
    call :runTestcase load-store
    call :runTestcase to-lower-case
    call :runTestcase movsx
    call :runTestcase unalign
    call :runTestcase shuixianhua
    call :runTestcase recursion
    call :runTestcase matrix-mul
GOTO :EOF

:begin
if "%~1" == "" (
    echo ERROR: Please specify valid testcase or benchmark name!
) else (
    if "%~1" == "all" (
        call :runTestcase memcpy
        call :runMediumTestcase
        call :runAdvancedTestcase
    ) else (
        if "%~1" == "medium" (
            call :runMediumTestcase
        ) else (
            if "%~1" == "advanced" (
                call :runAdvancedTestcase
            ) else (
                call :runTestcase %~1
            )
        )
    )
)
