# 本地调试

### 实验文件介绍

```
├── code_run //iverilog编译生成的二进制，运行脚本main.sh后会生成
├── submit
│   └── code.v //在该文件中，编写mips cpu的代码
├── test
│   ├── define.v
│   ├── ideal_mem.v //理想内存
│   ├── main.sh //评测脚本(Linux)
│   ├── main.bat //评测脚本(Windows)
│   ├── disassembly //反汇编软件程序源码（*.S），可读但不可修改,用于仿真调试时对照查看
│   ├── sim //存放benchmark。后缀为mem的文件是仿真测试激励文件，提取了可执行文件的代码段（.text）、数据段（.data）、未初始化数据段（.bss）等，以十六进制文本格式，按地址偏移存放，并在仿真时被自动加载到理想内存中。后缀为mem.log的文件，是评测用来对比的标准
│   └── testbench.v //实例化cpu和理想内存，
└── waveform // 存放波形文件
    └── out.vcd

```

### 本地运行评测脚本

进入mips_cpu文件夹，直接在该文件夹下运行main.sh，并附带上要测试的程序，比如下例的memcpy。运行main.sh脚本后，脚本会自动实例化你写的模块，并且与标准进行比对，告知错误结果，同时也会在waveform目录下生成波形out.vcd来查看波形。

Linux：
```shell
~$ cd mips_cpu
~/mips_cpu$ ./test/main.sh memcpy
```
Windows：
```shell
~$ cd mips_cpu
~/mips_cpu$ .\test\main.bat memcpy
```

如果通过测试样例，会显示如下界面：

![image-20220409102245692](https://csdn-imgsumbit.oss-cn-beijing.aliyuncs.com/img/image-20220409102245692.png)

如果报错会显示如下界面：

![image-20220409095348407](https://csdn-imgsumbit.oss-cn-beijing.aliyuncs.com/img/image-20220409095348407.png)

第一个红框，也就是testbench的warning。这个可以忽视，不会对结果有影响。如果是自己文件（code.v）出现了warning或者error，可能需要修改代码。

第二个红框，显示的是你的波形和标准波形不一样的地方。上面的信息提示你在**6290ns**处发生了错误，后面利用gtkwave来查看波形调试错误的时候，需要重点看6290ns处的波形。

评测脚本名字如下：

![image-20220409104736564](https://csdn-imgsumbit.oss-cn-beijing.aliyuncs.com/img/image-20220409104736564.png)

### gtkwave调试波形

还是在mips_cpu目录下，输入命令

```
gtkwave waveform/out.vcd
```

然后可以看到如下界面：

![image-20220409100151233](https://csdn-imgsumbit.oss-cn-beijing.aliyuncs.com/img/image-20220409100151233.png)

然后点击u_mips_cpu，可以看到信号列表列出了一堆信号。选择u_mips_cpu栏的信号，因为这部分是你实现的cpu的信号，其他信号都是testbench的相关信号。双击信号，即可在中间栏看到信号。然后点击菜单栏的➖适当缩放波形，把波形移到发生错误的时间。查看对应的信号调试错误。

![波形仿真](https://csdn-imgsumbit.oss-cn-beijing.aliyuncs.com/img/波形仿真.gif)



### 其他注意事项

1. 一定要在mips_cpu目录下，运行main.sh脚本。如果在test目录下运行，会报找不到文件的错误。因为脚本是使用相对路径来实现的。

2. 在命令行终端输入命令的时候，**巧用TAB键的自动补全**，可以提高效率。

3. 不要修改sim目录下的文件，会对本地评测造成影响。

4. 如果用powershell的时候提示iverilog找不到的错误，但是已经将iverilog加入到环境变量中了，请输入下面这一行命令。

```
$env:Path = [System.Environment]::GetEnvironmentVariable("Path","Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path","User")
```