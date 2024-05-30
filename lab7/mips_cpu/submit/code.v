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
			PC <= PCSrc == 2'b00 ? PCadd :
					 PCSrc == 2'b01 ? PCBranch :
					 PCSrc == 2'b10 ? PCJR : PCJA;
		end
	end

	wire [31:0]PCadd = PC + 4;
	wire [31:0]Signimm = (op == 6'b001111) ? {Instruction[15:0], 16'b0} : //lui
							{{16{Instruction[15]}}, Instruction[15:0]};
	wire [31:0]PCBranch = PCadd + (Signimm << 2);
	wire [31:0]PCJR = RF_rdata1;
	wire [31:0]PCJA = {PCadd[31:28], Instruction[25:0], 2'b00};

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
	wire [4:0]		RF_waddr = (RegDst == 0) ? rt :
								((op == 6'b000000 && func == 6'b001001 && rd == 5'b00000) || op == 6'b000011) ? 5'b11111 : //jalr jr
								 rd;
	wire [4:0]		RF_raddr1 = rs;
	wire [4:0]		RF_raddr2 = rt;
	wire			RF_wen;
	wire [31:0]		RF_wdata = (MemtoReg == 2'b00) ? ALU_Result :
									(MemtoReg == 2'b01) ? Read_data :
									(MemtoReg == 2'b10) ? PCadd + 4 :
									0;
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
	wire [31:0] ALU_A = ALUsa ? sa : RF_rdata1;
	wire [31:0] ALU_B = ALUSrc ? Signimm : RF_rdata2;
	wire [3:0]  ALU_ALUop;
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

	//控制器
	//wire ALU_Zero;
	wire [1:0] MemtoReg;
	//output MemWrite
	//wire [3:0]  ALU_ALUop;
	wire ALUSrc;
	//wire			RF_wen;
	wire RegDst;
	wire [1:0] PCSrc;
	wire ALUsa;

	controller Controller(
		.Instruction(Instruction),
		.Zero(ALU_Zero),
		.MemtoReg(MemtoReg),
		.MemWrite(MemWrite),
		.ALUop(ALU_ALUop),
		.ALUSrc(ALUSrc),
		.RegWrite(RF_wen),
		.RegDst(RegDst),
		.PCSrc(PCSrc),
		.ALUsa(ALUsa)
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
	input [3:0] ALUop,
	output Overflow,
	output CarryOut,
	output Zero,
	output [`DATA_WIDTH - 1:0] Result,
	output signal
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

    //零及其他标志信号
    assign Zero = (ALUop == 4'b0101 && Result_S == 0) ? 1 : //减
					(ALUop == 4'b1000 && B != 0) ? 1 : //movn
					(ALUop == 4'b1001 && B == 0) ? 1 : //movz
					(ALUop == 4'b0110 && Result_Slt == 1) ? 1 : //slt
					(ALUop == 4'b0111 && Result_Sltu == 1) ? 1 : //sltu
					0; //默认为0


    //根据操作码对Result赋值
    wire [`DATA_WIDTH - 1:0] Result_And, Result_Or, Result_Add, Result_Sub, Result_Slt, Result_Sltu, Result_Nor, Result_Xor,
							Result_Movn, Result_Movz, Result_Sll, Result_Srl, Result_Sra;
    assign Result_And = A & B;
    assign Result_Or = A | B;
    assign Result_Add = Result_S[`DATA_WIDTH - 1:0];
    assign Result_Sub = Result_S[`DATA_WIDTH - 1:0];
    assign Result_Slt = (Result_S[`DATA_WIDTH - 1] & ~Overflow_Sub) | (Overflow_Sub & ~Result_S[`DATA_WIDTH - 1]);
	assign Result_Sltu = Result_S[`DATA_WIDTH];
	assign Result_Nor = ~(A | B);
	assign Result_Xor = A ^ B;
	assign Result_Mov = A;
	assign Result_Sll = B << A;
	assign Result_Srl = B >> A;
	assign Result_Sra = $signed(B) >>> A;
    assign Result = (ALUop == 4'b0000) ? Result_Add : //加
                (ALUop == 4'b0001) ? Result_And : //与
                (ALUop == 4'b0010) ? Result_Or : //或
                (ALUop == 4'b0011) ? Result_Xor : //异或
                (ALUop == 4'b0100) ? Result_Nor : //或非
                (ALUop == 4'b0101) ? Result_Sub : //减
                (ALUop == 4'b0110) ? Result_Slt : //有符号小于
                (ALUop == 4'b0111) ? Result_Sltu : //无符号小于
				(ALUop == 4'b1000 && Zero == 1) ? Result_Mov : //movn
				(ALUop == 4'b1001 && Zero == 1) ? Result_Mov : //movz
				(ALUop == 4'b1010) ? Result_Sll : //逻辑左移
				(ALUop == 4'b1011) ? Result_Srl : //逻辑右移
				(ALUop == 4'b1100) ? Result_Sra : //算术右移
                32'b0; //默认为0
endmodule

//控制器
module controller(
	//输入指令字
	input [`DATA_WIDTH - 1:0] Instruction,
	input Zero,
	//输出控制信号
	output [1:0] MemtoReg,
	output MemWrite,
	output [3:0] ALUop,
	output ALUSrc,
	output RegWrite,
	output RegDst,
	output [1:0] PCSrc,
	output ALUsa
);
	wire Branch;
	//ALUctrl
	wire ALU_And, ALU_Or, ALU_Sub, ALU_Slt, ALU_Sltu, ALU_Nor, ALU_Xor, ALU_Movn, ALU_Movz, ALU_Sll, ALU_Srl, ALU_Sra;
	//and andi
	assign ALU_And = ((Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b100100)) || (Instruction[31:26] == 6'b001100);
	//or ori
	assign ALU_Or = ((Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b100101)) || (Instruction[31:26] == 6'b001101);
	//nor
	assign ALU_Nor = (Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b100111);
	//xor xori
	assign ALU_Xor = ((Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b100110)) || (Instruction[31:26] == 6'b001110);
	//slt slti
	assign ALU_Slt = ((Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b101010)) || (Instruction[31:26] == 6'b001010);
	//sltu sltiu
	assign ALU_Sltu = ((Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b101011)) || (Instruction[31:26] == 6'b001011);
	//subu beq bne bgez bgtz blez bltz movn movz
	assign ALU_Sub = ((Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b100011)) || (Instruction[31:26] == 6'b000100) 
					|| (Instruction[31:26] == 6'b000101) || (Instruction[31:26] == 6'b000001) || (Instruction[31:26] == 6'b000111) 
					|| (Instruction[31:26] == 6'b000110) || (Instruction[31:26] == 6'b000010) || (Instruction[31:26] == 6'b000011);
	assign ALU_Movn = (Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b001011);
	assign ALU_Movz = (Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b001010);
	assign ALU_Sll = (Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b000000);
	assign ALU_Srl = (Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b000010);
	assign ALU_Sra = (Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b000011);
	assign ALUop = (ALU_And) ? 4'b0001 : //and andi
					(ALU_Or) ? 4'b0010 : //or ori
					(ALU_Xor) ? 4'b0011 : //xor xori
					(ALU_Nor) ? 4'b0100 : //nor
					(ALU_Sub) ? 4'b0101 : //subu beq bne bgez bgtz blez bltz movn movz
					(ALU_Slt) ? 4'b0110 : //slt slti
					(ALU_Sltu) ? 4'b0111 : //sltu sltiu
					(ALU_Movn) ? 4'b1000 : //movn
					(ALU_Movz) ? 4'b1001 : //movz
					(ALU_Sll) ? 4'b1010 : //sll
					(ALU_Srl) ? 4'b1011 : //srl
					(ALU_Sra) ? 4'b1100 : //sra
					4'b0000; //默认为add
	assign ALUsa = (Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b000000) ? 1 ://sll
					(Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b000010) ? 1 ://srl
					(Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b000011) ? 1 ://sra
					0; //默认为0
	

	//控制信号
	//jalr jal
	assign MemtoReg = (Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b001001) ? 2'b10 : //jalr
						(Instruction[31:26] == 6'b000011) ? 2'b10 : //jal
						(Instruction[31:26] == 6'b100011) || (Instruction[31:26] == 6'b101011) ? 2'b01 : //lw sw
						2'b00; //默认为ALU_Result
	assign MemWrite = (Instruction[31:26] == 6'b101011) ? 1 : 0;
	assign Branch = (Instruction[31:26] == 6'b000100) || (Instruction[31:26] == 6'b000101);
	//andi ori xori addiu slti sltiu lui
	assign ALUSrc = (Instruction[31:26] == 6'b001100) || (Instruction[31:26] == 6'b001101) || (Instruction[31:26] == 6'b001110) 
					|| (Instruction[31:26] == 6'b001001) || (Instruction[31:26] == 6'b001010) || (Instruction[31:26] == 6'b001011) 
					|| (Instruction[31:26] == 6'b001111) || (Instruction[31:26] == 6'b100011) || (Instruction[31:26] == 6'b101011) ? 1 : 0;
	//movn movz slt sltu
	assign RegWrite = (Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b001011) ? Zero : //movn
						(Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b001010) ? Zero : //movz
						//jr j beq bne
						(Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b001000) ? 0 : //jr
						(Instruction[31:26] == 6'b000010) ? 0 : //j
						(Instruction[31:26] == 6'b000100) ? 0 : //beq
						(Instruction[31:26] == 6'b000101) ? 0 : //bne
						(Instruction[31:26] == 6'b101011) ? 0 : //sw
						1; //默认为1
	//r,j指令为1，i指令为0
	assign RegDst = (Instruction[31:26] == 6'b000000) ? 1 :
					(Instruction[31:26] == 6'b000010) ? 1 :
					(Instruction[31:26] == 6'b000011) ? 1 : 0;
	//jr jalr bne beq
	assign PCSrc = (Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b001000) ? 2'b10 : //jr
					(Instruction[31:26] == 6'b000000) && (Instruction[5:0] == 6'b001001) ? 2'b10 : //jalr
					(Instruction[31:26] == 6'b000010) ? 2'b11 : //j
					(Instruction[31:26] == 6'b000011) ? 2'b11 : //jal
					(Instruction[31:26] == 6'b000101) && (Zero == 0) ? 2'b01 : //bne
					(Instruction[31:26] == 6'b000100) && (Zero == 1) ? 2'b01 : //beq
					2'b00; //默认为PC+4
	
	assign Write_strb = 4'b1111;
	assign MemRead = Instruction[31:26] == 6'b100011;
endmodule
