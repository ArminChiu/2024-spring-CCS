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

// 成功，向地址 0x80008f00 写入 \x66\x66\x66\x66
void success()
{
    __asm(
        "lui  $8,0x6666\n"
        "ori  $8,$8,0x6666\n"
        "lui	$9,0x8000\n"
        "addiu	$9,$9,0x8f00\n"
        "sw $8, 0($9)\n"
    );
    return;
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


void copy(char *input)
{
    char buffer[16];
    
    // 获取 input 字符串长度，并将 input 存入 buffer。*溢出点
    int input_len = get_input(input, buffer);

    char *target_addr = (char *)0x80008f60;

    if (input_len == 0){
        return;
    }

    // 将 buffer 拷贝到 0x80008f60
    memcpy(target_addr, buffer, input_len);

    // 判断是否拷贝成功
    int flag = 1;
    for (int i = 0; i < input_len; i++){
        if (input[i] != target_addr[i]){
            flag = 0;
            break;
        }
    }

    if (flag){
        success();
    }else{
        fail();
    }
}

int main()
{
    // 作为输入
    char *input = @@;

    copy(input);
    
    return 0;
}


// 注入恶意代码，供参考
// void inject()
// {
//     __asm(
//         "lui  $8,0x4444\n"
//         "ori  $8,$8,0x4444\n"
//         "lui	$9,0x8000\n"
//         "addiu	$9,$9,0x8f30\n"
//         "sw $8, 0($9)\n"
//     );
// }

// ROP 配件
void gadget()
{
    __asm(
        "lui  $8,0x5555\n"
        "ori  $8,$8,0x5555\n"
        "lui	$9,0x8000\n"
        "addiu	$9,$9,0x8f34\n"
        "sw $8, 0($9)\n"
        "j _halt\n"
    );

}