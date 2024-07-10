import argparse

RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
RESET = '\033[0m'

class AttackChecker:
    def __init__(self, attack, test_cases, memdump_dir, use_color):
        self.attack = attack
        self.test_cases = test_cases
        self.memdump_dir = memdump_dir
        self.use_color = use_color

    def check_file_for_string(self, file_path, target_string):
        try:
            with open(file_path, 'r') as file:
                for line in file:
                    if target_string.startswith('!'):
                        if target_string[1:] not in line:
                            return True
                    elif target_string in line:
                        return True
        except FileNotFoundError:
            self.print_message(f"File not found: {file_path}", YELLOW)
        return False

    def check_defenses(self, test_case, attack_type, success_patterns, defense_patterns, normal_patterns):
        memdump_path = self.memdump_dir

        # Check if the attack succeeded
        attack_succeeded = True
        for pattern in success_patterns:
            if len(pattern) < 20:
                file_name = f'mem_dump_sp_{test_case}-{attack_type}.hex'
            else:
                file_name = f'mem_dump_{test_case}-{attack_type}.hex'

            if not self.check_file_for_string(f'{memdump_path}/{file_name}', pattern):
                attack_succeeded = False
                break

        if attack_succeeded:
            self.print_message(f"[{test_case}-{attack_type}]: {attack_type} Attack Succeeded!", RED)
            return

        # Check defenses if attack did not succeed
        defenses_working = []
        for defense, pattern in defense_patterns.items():
            if self.check_file_for_string(f'{memdump_path}/mem_dump_{test_case}-{attack_type}.hex', pattern):
                if attack_type == 'normal':
                    self.print_message(f"[{test_case}-{attack_type}]: {defense} should not work.", RED)
                else:
                    self.print_message(f"[{test_case}-{attack_type}]: {defense} Works!", GREEN)
                defenses_working.append(defense)
        
        # Check if execution is normal
        normal_flag = True
        if not defenses_working:
            for pattern in normal_patterns:
                if not self.check_file_for_string(f'{memdump_path}/mem_dump_{test_case}-{attack_type}.hex', pattern):
                    normal_flag = False
                    break
            if normal_flag:
                self.print_message(f"[{test_case}-{attack_type}]: Executed Normally!", GREEN)
            else:
                self.print_message(f"[{test_case}-{attack_type}]: Executed abnormally, while none of defense works.", YELLOW)


    def check_inject(self, test_case):
        if test_case == 'copy':
            success_patterns = ['0x80008f30: 44444444']
            normal_patterns = ['0x80008f00: 66666666']
        elif test_case == 'password':
            success_patterns = ['0x80008f00: 66666666', '!0x80008fc4: 80000']
            normal_patterns = ['0x80008f04: ffffffff']
        elif test_case == 'select':
            success_patterns = ['0x80008f10: 11111111', '0x80008f14: 22222222', '0x80008f18: 33333333', '!0x80008fc4: 80000']
            normal_patterns = ['0x80008f04: ffffffff']
        defense_patterns = {
            'NX': '0x80008f20: aaaaaaaa',
            'Shadow Stack': '0x80008f24: bbbbbbbb',
            'CFI': '0x80008f28: cccccccc'
        }

        print("---------------------------------------------------------------------------------")
        self.check_defenses(test_case, 'inject', success_patterns, defense_patterns, normal_patterns)
        self.check_defenses(test_case, 'normal', success_patterns, defense_patterns, normal_patterns)
        

    def check_rop(self, test_case):
        if test_case == 'copy':
            success_patterns = ['0x80008f34: 55555555']
            normal_patterns = ['0x80008f00: 66666666']
        elif test_case == 'password':
            success_patterns = ['0x80008f00: 66666666', '0x80008fc4: 80000']
            normal_patterns = ['0x80008f04: ffffffff']
        elif test_case == 'select':
            success_patterns = ['0x80008f10: 11111111', '0x80008f14: 22222222', '0x80008f18: 33333333', '0x80008fc4: 80000']
            normal_patterns = ['0x80008f04: ffffffff']
        defense_patterns = {
            'Shadow Stack': '0x80008f24: bbbbbbbb',
            'CFI': '0x80008f28: cccccccc'
        }
        print("---------------------------------------------------------------------------------")
        self.check_defenses(test_case, 'rop', success_patterns, defense_patterns , normal_patterns)
        self.check_defenses(test_case, 'normal', success_patterns, defense_patterns, normal_patterns)
        

    def check_jop(self, test_case):
        success_patterns = ['0x80008f00: 66666666', '0x80008fcc: 800001']
        defense_patterns = {
            'CFI': '0x80008f28: cccccccc'
        }
        normal_patterns = ['0x80008f04: ffffffff']

        print("---------------------------------------------------------------------------------")
        self.check_defenses(test_case, 'jop', success_patterns, defense_patterns, normal_patterns)
        self.check_defenses(test_case, 'normal', success_patterns, defense_patterns, normal_patterns)
        

    def print_message(self, message, color):
        if self.use_color:
            print(color + message + RESET)
        else:
            print(message)

def main():
    parser = argparse.ArgumentParser(description='Check for different types of attacks on various test cases.')
    parser.add_argument('attack', choices=['inject', 'rop', 'jop'], help='Type of attack to check for')
    parser.add_argument('test_case', nargs='?', choices=['copy', 'password', 'select'], help='Test case to check against')
    parser.add_argument('--memdump_dir', default='test/mem_dump', help='Directory containing the memory dump files')
    parser.add_argument('--no-color', action='store_true', help='Disable colored output')
    args = parser.parse_args()

    test_cases = [args.test_case] if args.test_case else ['copy', 'password', 'select']
    checker = AttackChecker(args.attack, test_cases, args.memdump_dir, not args.no_color)

    for test_case in test_cases:
        if args.attack == 'inject':
            checker.check_inject(test_case)
        elif args.attack == 'rop':
            checker.check_rop(test_case)
        elif args.attack == 'jop' and test_case == 'password':
            checker.check_jop(test_case)

if __name__ == '__main__':
    main()
