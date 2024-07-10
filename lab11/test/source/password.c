#include "trap.h"
#define KEY_SIZE 32

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
int get_input(char* input, char* dest) {
    
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

int cmp_key_full(char *buffer, char *key)
{
    int cmp_result = 1;
    for (int i = 0; i < KEY_SIZE; i++)
    {
        if (buffer[i] != key[i])
        {
            cmp_result = 0;
            break;
        }
    }
    return cmp_result;
}

int cmp_key_fast(char *buffer, char *key)
{
    int cmp_result = 1;
    for (int i = 0; i < 4; i++)
    {
        if (buffer[i] != key[i])
        {
            cmp_result = 0;
            break;
        }
    }
    return cmp_result;
}

int check(char *input, int (*cmp_key_func)())
{
    char buffer[KEY_SIZE] = "\xee\xee\xee\xee\xee\xee\xee\xee"
    "\xee\xee\xee\xee\xee\xee\xee\xee"
    "\xee\xee\xee\xee\xee\xee\xee\xee"
    "\xee\xee\xee\xee\xee\xee\xee\xee";
    
    // 正确的 password
    char key[KEY_SIZE] = "\x88\x77\x66\x55\x44\x33\x22\x11"
    "\x88\x77\x66\x55\x44\x33\x22\x11"
    "\x88\x77\x66\x55\x44\x33\x22\x11"
    "\x88\x77\x66\x55\x44\x33\x22\x11";

    get_input(input, buffer);

    return cmp_key_func(buffer, key);
}

int main()
{
    // 输入
    char *input = @@
    ;

    // 函数指针，用于 JOP 攻击
    int (*cmp_key_func)()  = NULL;
    
    cmp_key_func = cmp_key_full;

    if (check(input, cmp_key_func))
    {
        success();
    } else {
        fail();
    }
    
    return 0;
}

