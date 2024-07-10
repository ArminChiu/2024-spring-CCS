import argparse

def replace_shellcode(shellcode_file, c_file, output_file):
    # 读取shellcode文件内容
    with open(shellcode_file, 'r') as sf:
        shellcode = sf.read()

    # 读取C代码文件内容
    with open(c_file, 'r') as cf:
        c_code = cf.read()

    # 替换C代码中的@@
    replaced_code = c_code.replace('@@', shellcode)

    # 将结果写入输出文件
    with open(output_file, 'w') as of:
        of.write(replaced_code)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Replace @@ in C code with shellcode')
    parser.add_argument('shellcode_file', type=str, help='Path to the shellcode file')
    parser.add_argument('c_file', type=str, help='Path to the C code file')
    parser.add_argument('output_file', type=str, help='Path to the output file')

    args = parser.parse_args()

    replace_shellcode(args.shellcode_file, args.c_file, args.output_file)
