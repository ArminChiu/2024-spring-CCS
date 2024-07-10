import argparse

def main():
    parser = argparse.ArgumentParser(description='Beautify a hex dump file.')
    parser.add_argument('file_path', type=str, help='Path to the input hex dump file')
    parser.add_argument('save_file_path', type=str, help='Path to save the beautified hex dump file')
    args = parser.parse_args()

    file_path = args.file_path
    save_file_path = args.save_file_path

    mem_list = []
    with open(file_path, 'r') as file:
        for line in file:
            # 忽略 //的行
            if line.startswith('//'):
                continue
            mem_list.append(line.strip())

    # 起始地址
    addr = 0x80000000
    beautify_mem = []
    start_mem = False

    for mem in mem_list:
        if 'xxxxxxxx' in mem and not start_mem:
            start_mem = True
            addr = 0x80008ffc - (1024 - int((addr - 0x80000000) / 4) - 1) * 4

        beautify_mem.append(f'0x{addr:08x}: {mem}')
        addr += 4

    # 保存到文件
    with open(save_file_path, 'w') as file:
        for mem in beautify_mem:
            file.write(mem + '\n')

if __name__ == '__main__':
    main()
