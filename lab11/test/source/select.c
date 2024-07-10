#include "trap.h"

// NX bit 防御成功
void success_nx_bit()
{
    __asm(
        "lui  $8,0xaaaa\n"
        "ori  $8,$8,0xaaaa\n"
        "lui	$9,0x8000\n"
        "addiu	$9,$9,0x8f20\n"
        "sw $8, 0($9)\n"
        "j   _halt\n"
    );
}

// 影子栈防御成功
void success_shadow_stack()
{
    __asm(
        "lui  $8,0xbbbb\n"
        "ori  $8,$8,0xbbbb\n"
        "lui	$9,0x8000\n"
        "addiu	$9,$9,0x8f24\n"
        "sw $8, 0($9)\n"
        "j   _halt\n"
    );
}

// CFI 防御成功
void success_cfi()
{
    __asm(
        "lui  $8,0xcccc\n"
        "ori  $8,$8,0xcccc\n"
        "lui	$9,0x8000\n"
        "addiu	$9,$9,0x8f28\n"
        "sw $8, 0($9)\n"
        "j   _halt\n"
    );
}

void exit(){
    __asm(
        "addiu $0,$0,0\n"
    );
}

// 失败，向地址 0x80008f04 写入 \xff\xff\xff\xff
void fail()
{
    __asm(
        "lui  $8,0xffff\n"
        "ori  $8,$8,0xffff\n"
        "lui	$9,0x8000\n"
        "addiu	$9,$9,0x8f04\n"
        "sw $8, 0($9)\n"
    );
    return;
}

void func_1()
{
    __asm(
        "lui  $8,0x1111\n"
        "ori  $8,$8,0x1111\n"
        "lui	$9,0x8000\n"
        "addiu	$9,$9,0x8f10\n"
        "sw $8, 0($9)\n"
    );
    exit();
}



void func_2()
{
    __asm(
        "lui  $8,0x2222\n"
        "ori  $8,$8,0x2222\n"
        "lui	$9,0x8000\n"
        "addiu	$9,$9,0x8f14\n"
        "sw $8, 0($9)\n"
    );
    exit();
}

void func_3()
{
    __asm(
        "lui  $8,0x3333\n"
        "ori  $8,$8,0x3333\n"
        "lui	$9,0x8000\n"
        "addiu	$9,$9,0x8f18\n"
        "sw $8, 0($9)\n"
    );
    exit();
}


// 拷贝，并计算输入的字符串长度
int get_input(char* input, char* dest)
{
    
    int len = 0;

    while (1) {
        if (input[len] == '\0' && input[len+1] == '\0' && input[len+2] == '\0' && input[len+3] == '\0'){
            break;
        }

        dest[len] = input[len];

        len++;
    }

    return len;
}

int parse(char *input)
{
    char buf[1];
    get_input(input, buf);

    if (buf[0] == '1'){
        return 1;
    }else if (buf[0] == '2'){
        return 2;
    }else if (buf[0] == '3'){
        return 3;
    }

    return -1;
}


int main()
{
    // 输入
    char *input = @@
    ;

    int i = parse(input);

    if (i == 1)
    {   
        func_1();
    }else if (i == 2){
        func_2();
    }else if (i == 3){
        func_3();
    }else{
        fail();
    }
    
    return 0;
}
