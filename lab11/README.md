# README

[toc]

## 实验要求

### 不可执行位

在取指时，判断指令地址是否处于合法指令地址范围以内。如果不是，则认定为检测到攻击。

检测到攻击后，需要将PC置为32‘h80000068，也就是源码中的success_nx_bit函数地址，便于评测。

### 影子栈

执行**jal, jalr**时，将返回地址拷贝到影子栈
执行**jr ra**时，将影子栈返回地址取回与ra进行对比。如果不一致，则认定为检测到攻击。

检测到攻击后，需要将PC置为32‘h800000a8，也就是源码中的success_shadow_stack函数地址，便于评测。

### 粗粒度CFI

执行jalr时，判断跳转目标的指令是否为nop指令。如果不是，则认定为检测到攻击。

检测到攻击后，需要将PC置为32‘h800000e8，也就是源码中的success_cfi函数地址，便于评测。

## 实验文件介绍

```mark
├── code_run //iverilog编译生成的二进制，运行脚本main.sh后会生成
├── main.sh //评测脚本
├── shellcode
│   ├── copy-inject //在该文件中，编写针对copy程序的代码注入攻击 shellcode
│   ├── copy-rop //在该文件中，编写针对copy程序的ROP攻击 shellcode
│   └── ...
├── source
│   └── code.v //在该文件中，编写CPU代码
├── test
│   ├── define.v
│   ├── ideal_mem.v //理想内存
│   ├── mem_dump_[程序名]-[攻击名].hex //执行程序后dump出来的理想内存的值
│   ├── disassembly //反汇编软件程序源码（*.S），可读但不可修改,用于仿真调试时对照查看
│   ├── source //软件程序源码，可读但不可修改,用于理解实验原理。包含copy.c, password.c, select.c。
│   ├── sim //存放benchmark。后缀为mem的文件是仿真测试激励文件，提取了可执行文件的代码段（.text）、数据段（.data）、未初始化数据段（.bss）等，以十六进制文本格式，按地址偏移存放，并在仿真时被自动加载到理想内存中。
│   └── testbench.v //实例化cpu和理想内存，影子栈。
└── waveform // 存放波形文件
    └── [程序名]-[攻击名].vcd
```

### 本地运行评测脚本

进入mips_cpu_defense文件夹，直接在该文件夹下运行main.sh，并附带上要测试的攻击程序：

- inject：用于测试不可执行位（NX）的防御机制的实现。
- rop：用于测试影子栈（shadow stack）防御机制的实现
- jop：用于测试粗粒度控制流完整性（CFI）防御机制的实现。

运行main.sh脚本后，脚本会自动实例化你写的模块，并且与标准进行比对，告知错误结果，同时也会在waveform目录下生成波形out.vcd来查看波形。

```shell
~$ cd mips_cpu_defense
~/mips_cpu_defense$ ./main.sh inject
~/mips_cpu_defense$ ./main.sh rop
~/mips_cpu_defense$ ./main.sh jop
```



## 其他注意事项

1. 一定要在mips_cpu_defense目录下，运行main.sh脚本。如果在test目录下运行，会报找不到文件的错误。因为脚本是使用相对路径来实现的。
2. 在命令行终端输入命令的时候，**巧用TAB键的自动补全**，可以提高效率。
3. 不要修改sim目录下的文件，会对本地评测造成影响。
4. 评测某个防御机制时，最好将其他防御机制的异常处理部分的代码注释掉（检测到攻击后将PC置为指定值）