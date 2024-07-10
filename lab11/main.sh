#!/bin/bash


AM_HOME="$PWD/utils/nexus-am"
export AM_HOME=$AM_HOME
MAIN_DIR="$PWD"
ATTACK=$1
build_testcase() {
    prog_name=${1}
    case_type=${2}
    case_name=${prog_name}-${case_type}
    # 1. replace shellcode in the c file
    source_c="./test/source/$prog_name.c"
    dst_c=$AM_HOME"/tests/myAMapp/tests/bug.c"  
    python3 utils/replace_shellcode.py "./shellcode/$case_name" $source_c $dst_c 
    # 2. build the binary via am
    pushd $AM_HOME"/tests/myAMapp" > /dev/null
    rm -rf ./build
    if [ $ATTACK == "jop" ];then
        make ARCH=mips32-npc
        # instrumentation
        sed -i '412a\    addiu\t$0,$0,0'  $AM_HOME/tests/myAMapp/build/mips32-npc/bug.S
        sed -i '499a\    addiu\t$0,$0,0'  $AM_HOME/tests/myAMapp/build/mips32-npc/bug.S
        sed -i '21 s/^/# /' $AM_HOME/Makefile.compile
        make ARCH=mips32-npc
        sed -i '21 s/^# //' $AM_HOME/Makefile.compile
    else
        make ARCH=mips32-npc 
        
        if [ $? -ne 0 ]; then
            echo "Build failed"
            exit 1
        fi
    fi
    popd > /dev/null
    
    # 3. convert the binary to mem file
    pushd $AM_HOME/tests/myAMapp/build/ > /dev/null
    xxd -c 4 -e bug-mips32-npc.bin | cut -c 11-18 > $case_name.mem
    mv $case_name.mem $MAIN_DIR"/test/sim"
    popd > /dev/null
    # 4. build and run the mips cpu
    echo "\`define INST_MEM \"test/sim/${case_name}.mem\"" >test/define.v
    echo "\`define CYCLE 10000" >> test/define.v
    echo "---------------------${case_name}---------------------"
    iverilog ./test/testbench.v ./source/code.v -o code_run
    vvp code_run && rm code_run
    cd $MAIN_DIR
    # 5. dump the memory
    mem_dump_file="mem_dump_$case_name.hex"
    python3 utils/beautify_mem.py mem_dump.hex  $MAIN_DIR/test/mem_dump/$mem_dump_file
    

    rm mem_dump.hex
    mv waveform/out.vcd waveform/${case_name}.vcd
    # 6. dump the memory_sp
    cd $MAIN_DIR
    if [ "$prog_name" = "password" ] ; then
        echo "\`define INST_MEM \"test/sim/${case_name}.mem\"" >test/define.v
        echo "\`define CYCLE 2940" >> test/define.v
        echo "---------------------${case_name}---------------------"
        iverilog ./test/testbench.v ./source/code.v -o code_run
        vvp code_run && rm code_run
        mem_dump_sp_file="mem_dump_sp_$case_name.hex"
        python3 utils/beautify_mem.py mem_dump.hex  $MAIN_DIR/test/mem_dump/$mem_dump_sp_file
        rm mem_dump.hex
    elif [ "$prog_name" = "select" ]; then
        echo "\`define INST_MEM \"test/sim/${case_name}.mem\"" >test/define.v
        echo "\`define CYCLE 1300" >> test/define.v
        echo "---------------------${case_name}---------------------"
        iverilog ./test/testbench.v ./source/code.v -o code_run
        vvp code_run && rm code_run
        mem_dump_sp_file="mem_dump_sp_$case_name.hex"
        python3 utils/beautify_mem.py mem_dump.hex  $MAIN_DIR/test/mem_dump/$mem_dump_sp_file
        #python3 utils/beautify_mem.py mem_dump.hex  ./test/mem_dump/mem_dump_sp_select-inject.hex  
        rm mem_dump.hex
    fi
}

# judge the result

if [ "$ATTACK" == "inject" ]; then
    build_testcase "copy" $ATTACK
    build_testcase "password" $ATTACK
    build_testcase "select" $ATTACK
    build_testcase "copy" "normal"
    build_testcase "password" "normal"
    build_testcase "select" "normal"
elif [ "$ATTACK" == "rop" ]; then
    build_testcase "copy" $ATTACK
    build_testcase "password" $ATTACK
    build_testcase "select" $ATTACK
    build_testcase "copy" "normal"
    build_testcase "password" "normal"
    build_testcase "select" "normal"
elif [ "$ATTACK" == "jop" ]; then
    build_testcase "password" $ATTACK
    build_testcase "password" "normal"
else
    echo "Unknown Attack: $1"
    exit 1
fi


python3 $MAIN_DIR/utils/judge.py $ATTACK 


