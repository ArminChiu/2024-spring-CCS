`timescale 10ns / 1ns
`define DATA_WIDTH 32
`define ADDR_WIDTH 5

module top_module(
	input  rst, //复位信号
	input  clk, //时钟信号
	output reg [31:0] PC, //程序计数器
	input  [31:0] Instruction, //指令
	output [31:0] Address, //存储器地址
	output MemWrite, //存储器写使能
	output [31:0] Write_data, //存储器写数据
	output [3:0] Write_strb, //存储器写掩码
	input  [31:0] Read_data, //存储器读数据
	output MemRead //存储器读使能
);	
	//顶层输出
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			PC <= 0;
		end else begin
			PC <= PCSrc ? PCBranch : PC + 4;
		end
	end
	
	wire [31:0]Signimm = {{16{Instruction[15]}}, Instruction[15:0]}; 
	wire [31:0]PCBranch = PC + (Signimm << 2) + 4;

	assign Address = ALU_Result;
	assign Write_data = RF_rdata2;
	assign Write_strb = 4'b1111;
	assign MemRead = Instruction[31:26] == 6'b100011;
	

	//R类型
	wire [5:0] op = Instruction[31:26];
	wire [4:0] rs = Instruction[25:21];
	wire [4:0] rt = Instruction[20:16];
	wire [4:0] rd = Instruction[15:11];
	wire [4:0] sa = Instruction[10:6];
	wire [5:0] func = Instruction[5:0];
	//I类型
	//wire [5:0] op;
	//wire [4:0] rs;
	//wire [4:0] rt;
	wire [15:0] imm = Instruction[15:0];
	//J类型
	//wire [5:0] op;
	wire [25:0] addr = Instruction[25:0];

	//寄存器堆
	wire [4:0]		RF_waddr = RegDst ? rd : rt;
	wire [4:0]		RF_raddr1 = rs;
	wire [4:0]		RF_raddr2 = rt;
	wire			RF_wen;
	wire [31:0]		RF_wdata = MemtoReg ? Read_data : ALU_Result;
	wire [31:0]		RF_rdata1;
	wire [31:0]		RF_rdata2;

	reg_file RF(
		.clk(clk),
		.rst(rst),
		.waddr(RF_waddr),
		.raddr1(RF_raddr1),
		.raddr2(RF_raddr2),
		.wen(RF_wen),
		.wdata(RF_wdata),
		.rdata1(RF_rdata1),
		.rdata2(RF_rdata2)
	);

	//ALU
	wire [31:0] ALU_A = RF_rdata1;
	wire [31:0] ALU_B = ALUSrc ? Signimm : RF_rdata2;
	wire [2:0]  ALU_ALUop;
	wire 		ALU_Overflow;
	wire 		ALU_CarryOut;
	wire 		ALU_Zero;
	wire [31:0] ALU_Result;	

	alu ALU(
		.A(ALU_A),
		.B(ALU_B),
		.ALUop(ALU_ALUop),
		.Overflow(ALU_Overflow),
		.CarryOut(ALU_CarryOut),
		.Zero(ALU_Zero),
		.Result(ALU_Result)
	);

	//移位器
	wire [31:0] Shift_A = RF_rdata2;
	wire [4:0] Shift_B = sa;
	wire [1:0]  Shift_Shiftop = func[5:4];
	wire [31:0] Shift_Result;

	shifter Shift(
		.A(Shift_A),
		.B(Shift_B),
		.Shiftop(Shift_Shiftop),
		.Result(Shift_Result)
	);

	//控制器
	//wire ALU_Zero;
	wire MemtoReg;
	//output MemWrite
	wire Branch;
	//wire [2:0]  ALU_ALUop;
	wire ALUSrc;
	//wire			RF_wen;
	wire RegDst;
	wire PCSrc;

	controller Controller(
		.Instruction(Instruction),
		.Zero(ALU_Zero),
		.MemtoReg(MemtoReg),
		.MemWrite(MemWrite),
		.Branch(Branch),
		.ALUop(ALU_ALUop),
		.ALUSrc(ALUSrc),
		.RegWrite(RF_wen),
		.RegDst(RegDst),
		.PCSrc(PCSrc)
	);								
					
endmodule

//寄存器堆
module reg_file(
	input clk,
	input rst,
	input [`ADDR_WIDTH - 1:0] waddr, //数据写 地址
	input [`ADDR_WIDTH - 1:0] raddr1, //指令读 地址
	input [`ADDR_WIDTH - 1:0] raddr2, //数据读 地址
	input wen,
	input [`DATA_WIDTH - 1:0] wdata, //数据写 数据
	output [`DATA_WIDTH - 1:0] rdata1, //指令读 数据
	output [`DATA_WIDTH - 1:0] rdata2 //数据读 数据
);

	//寄存器堆定义
	reg [`DATA_WIDTH - 1:0] mem [0:2**`ADDR_WIDTH - 1];

    //异步读(0号寄存器始终为0)
	assign rdata1 = (raddr1 == 5'b0)? 32'b0:mem[raddr1];
	assign rdata2 = (raddr2 == 5'b0)? 32'b0:mem[raddr2];

    //同步写
    always @(posedge clk) begin
        if(wen) begin
            //0号寄存器不可写
            if(waddr != 0) begin
                mem[waddr] <= wdata;
            end
        end
    end

endmodule

//ALU
module alu(
	input [`DATA_WIDTH - 1:0] A,
	input [`DATA_WIDTH - 1:0] B,
	input [2:0] ALUop,
	output Overflow,
	output CarryOut,
	output Zero,
	output [`DATA_WIDTH - 1:0] Result
);

	//加减法条件判断 Result_S为加一符号位，仅在此处生成了一个加法器
    wire [`DATA_WIDTH:0] Result_S;
    assign Result_S = A + (ALUop[2] == 0 ? B : ~B + 1);

    //溢出
    wire Overflow_Add;
    wire Overflow_Sub;
    assign Overflow_Add = (A[`DATA_WIDTH - 1] == B[`DATA_WIDTH - 1]) && (A[`DATA_WIDTH - 1] != Result_S[`DATA_WIDTH - 1]);
    assign Overflow_Sub = (A[`DATA_WIDTH - 1] != B[`DATA_WIDTH - 1]) && (A[`DATA_WIDTH - 1] != Result_S[`DATA_WIDTH - 1]);
    assign Overflow = (ALUop[2] == 0) ? Overflow_Add : Overflow_Sub;

    //进借位
    assign CarryOut = Result_S[`DATA_WIDTH];

    //零
    assign Zero = (Result == 0);

    //根据操作码对Result赋值
    wire [`DATA_WIDTH - 1:0] Result_And, Result_Or, Result_Add, Result_Sub, Result_Less;
    assign Result_And = A & B;
    assign Result_Or = A | B;
    assign Result_Add = Result_S[`DATA_WIDTH - 1:0];
    assign Result_Sub = Result_S[`DATA_WIDTH - 1:0];
    assign Result_Less = (Result_S[`DATA_WIDTH - 1] & ~Overflow_Sub) | (Overflow_Sub & ~Result_S[`DATA_WIDTH - 1]);
    assign Result = (ALUop == 3'b000) ? Result_And :
                    (ALUop == 3'b001) ? Result_Or :
                    (ALUop == 3'b010) ? Result_Add :
                    (ALUop == 3'b110) ? Result_Sub :
                                        Result_Less;

endmodule

//移位器
module shifter (
	input [`DATA_WIDTH - 1:0] A,
	input [`ADDR_WIDTH - 1:0] B,
	input [1:0] Shiftop,
	output [`DATA_WIDTH - 1:0] Result
);

	//根据操作码对Result赋值
	assign Result = (Shiftop == 2'b00) ? A << B : //逻辑左移
					(Shiftop == 2'b01) ? A >> B : //逻辑右移
							($signed(A)) >>> B; //算术右移

endmodule

//控制器
module controller(
	//输入指令字
	input [`DATA_WIDTH - 1:0] Instruction,
	input Zero,
	//输出控制信号
	output MemtoReg,
	output MemWrite,
	output Branch,
	output [2:0]ALUop,
	output ALUSrc,
	output RegWrite,
	output RegDst,
	output PCSrc
);

	//addiu 001001 rs rt imm	rt = rs + imm
	//bne   000101 rs rt imm	if rs != rt then PC = PC + imm
	//lw    100011 rs rt imm	rt = M[rs + imm]
	//sw	101011 rs rt imm	M[rs + imm] = rt
	//sll 	000000 00000 rt rd sa 000000	rd = rt << sa
	//运算类指令
	wire addiu = (Instruction[31:26] == 6'b001001);
	//移位指令
	wire sll = (Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b000000);
	//跳转类指令
	wire bne = (Instruction[31:26] == 6'b000101);
	//访存类指令
	wire lw = (Instruction[31:26] == 6'b100011);
	wire sw = (Instruction[31:26] == 6'b101011);

	assign MemtoReg = lw;
	assign MemWrite = sw;
	assign Branch = bne;
	assign ALUop = addiu ? 3'b010 : sll ? 3'b000 : bne ? 3'b110 : 3'b010;
	assign ALUSrc = addiu ? 1'b1 : sll ? 1'b1 : bne ? 1'b0 : 1'b1;
	assign RegWrite = addiu ? 1'b1 : bne ? 1'b0 : lw ? 1'b1 : sw ? 1'b0 : sll ? 1'b1 : 1'b0;
	assign RegDst = sll ? 1'b1 : 1'b0;
	assign PCSrc = Branch && !Zero;

endmodule
