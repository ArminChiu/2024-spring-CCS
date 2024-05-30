#!/bin/bash
runTestcase() {
    testcase=${1}
    echo "\`define TRACE_FILE \"test/sim/${testcase}.mem.log\"" > test/define.v
    echo "\`define INST_MEM \"test/sim/${testcase}.mem\"" >> test/define.v
    echo "---------------------${testcase}---------------------"
    iverilog ./test/testbench.v ./submit/code.v -o code_run
    vvp code_run 

}

runMediumTestcase() {
    runTestcase "sum"
    runTestcase "mov-c"
    runTestcase "fib"
    runTestcase "add"
    runTestcase "if-else"
    runTestcase "pascal"
    runTestcase "quick-sort"
    runTestcase "select-sort"
    runTestcase "max"
    runTestcase "min"
    runTestcase "switch"
    runTestcase "bubble-sort"
}

runAdvancedTestcase() {
    #runTestcase "shuixianhua"
    runTestcase "sub-longlong"
    runTestcase "bit"
    #runTestcase "recursion"
    runTestcase "fact"
    runTestcase "add-longlong"
    runTestcase "shift"
    runTestcase "wanshu"
    runTestcase "goldbach"
    runTestcase "leap-year"
    runTestcase "prime"
    runTestcase "mul-longlong"
    runTestcase "load-store"
    runTestcase "to-lower-case"
    runTestcase "movsx"
    #runTestcase "matrix-mul"
    runTestcase "unalign"
}
if [ ! $1 ]; then
    echo "ERROR: Please specify vaild testcase or benchmark name!"
else 
    if [ $1 == "all" ]; then
        runTestcase "memcpy"
        runMediumTestcase
        runAdvancedTestcase
    else
        if [ $1 == 'medium' ]; then
            runMediumTestcase
        else 
            if [ $1 == 'advanced' ]; then
                runAdvancedTestcase
            else
                runTestcase ${1}
            fi
        fi
    fi
fi
