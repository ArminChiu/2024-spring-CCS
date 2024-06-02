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

	//以下是顶层连线

	//程序计数器PC
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			PC <= 0;
		end else begin
			PC <= PCSrc == 2'b00 ? PCadd : //PC+4
					 PCSrc == 2'b01 ? PCBranch : //分支
					 PCSrc == 2'b10 ? PCJR : PCJA; //跳转
		end
	end

	wire [31:0]PCadd = PC + 4; //顺序执行
	wire [31:0]Signimm = (op == 6'b001111) ? {Instruction[15:0], 16'b0} : //lui
						 (op[5:2] == 4'b0011) ? {16'b0, Instruction[15:0]} : //andi ori xori 无符号扩展
						 {{16{Instruction[15]}}, Instruction[15:0]}; //其他有符号扩展
	wire [31:0]PCBranch = PCadd + (Signimm << 2); //分支跳转
	wire [31:0]PCJR = RF_rdata1;
	wire [31:0]PCJA = {PCadd[31:28], Instruction[25:0], 2'b00};
	
	//存储器读写地址及数据
	assign Address = Address_out;
	assign Write_data = Data_out;

	//R类型指令
	wire [5:0] op = Instruction[31:26];
	wire [4:0] rs = Instruction[25:21];
	wire [4:0] rt = Instruction[20:16];
	wire [4:0] rd = Instruction[15:11];
	wire [4:0] sa = Instruction[10:6];
	wire [5:0] func = Instruction[5:0];


	//以下是子模块实例化

	//寄存器堆实例化
	wire [4:0]		RF_waddr = (RegDst == 0) ? rt :
							   ((op == 6'b000000 && func == 6'b001001 && rd == 5'b00000)|| op == 6'b000011) ? 5'b11111 : //jalr jr
							   rd;
	wire [4:0]		RF_raddr1 = rs;
	wire [4:0]		RF_raddr2 = (op == 6'b000001 && rt == 5'b00001) ? rt - 1 : rt;
	wire			RF_wen;
	wire [31:0]		RF_wdata = (MemtoReg == 2'b00) ? ALU_Result :
							   (MemtoReg == 2'b01) ? Data_out :
							   (MemtoReg == 2'b10) ? PCadd + 4 : 0;
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

	//ALU实例化
	wire [31:0] ALU_A = ALUsa ? sa : RF_rdata1;
	wire [31:0] ALU_B = ALUSrc ? Signimm : RF_rdata2;
	wire [3:0]  ALU_ALUop;
	wire 		ALU_Overflow;
	wire 		ALU_CarryOut;
	wire 		ALU_Zero;
	wire		ALU_Signal;
	wire [31:0] ALU_Result;	

	alu ALU(
		.A(ALU_A),
		.B(ALU_B),
		.ALUop(ALU_ALUop),
		.Overflow(ALU_Overflow),
		.CarryOut(ALU_CarryOut),
		.Zero(ALU_Zero),
		.Signal(ALU_Signal),
		.Result(ALU_Result)
	);

	//数据转换器实例化
	wire [31:0] Data_out;
	wire [31:0] Address_out;

	dataconverter Dataconverter(
		.Instruction(Instruction),
		.Address_in(ALU_Result),
		.Data_rf2(RF_rdata2),
		.Data_in(Read_data),
		.Address_out(Address_out),
		.Data_out(Data_out),
		.Write_strb(Write_strb)
	);

	//控制器实例化
	wire [1:0] MemtoReg;
	wire ALUSrc;
	wire RegDst;
	wire [1:0] PCSrc;
	wire ALUsa;

	controller Controller(
		.Instruction(Instruction),
		.Zero(ALU_Zero),
		.Signal(ALU_Signal),
		.MemtoReg(MemtoReg),
		.MemWrite(MemWrite),
		.ALUop(ALU_ALUop),
		.ALUSrc(ALUSrc),
		.RegWrite(RF_wen),
		.RegDst(RegDst),
		.PCSrc(PCSrc),
		.ALUsa(ALUsa),
		.MemRead(MemRead)
	);								
					
endmodule


//以下是子模块

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
	output Signal,
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

    //零及其他标志信号
    assign Zero = (ALUop == 4'b0101 && Result_S == 0) ? 1 : //减
				  (ALUop == 4'b0110 && A == 0) ? 1 : 0; //branch
	assign Signal = (ALUop[3:1] == 3'b100 && B == 0) ? 1 : //movn movz
					(ALUop == 4'b0110) ? Result_Slt : //slt
					(ALUop == 4'b0111) ? Result_Sltu : //sltu
					(ALUop == 4'b1101) ? !A[`DATA_WIDTH - 1] : //bgez
					0; //默认为0

    //根据操作码对Result赋值
    wire [`DATA_WIDTH - 1:0] Result_And, Result_Or, Result_Add, Result_Sub, Result_Slt, Result_Sltu,
							Result_Nor, Result_Xor, Result_Mov, Result_Sll, Result_Srl, Result_Sra;
    assign Result_And = A & B;
    assign Result_Or = A | B;
    assign Result_Add = Result_S[`DATA_WIDTH - 1:0];
    assign Result_Sub = Result_S[`DATA_WIDTH - 1:0];
    assign Result_Slt = (Result_S[`DATA_WIDTH - 1] & ~Overflow_Sub) | (Overflow_Sub & ~Result_S[`DATA_WIDTH - 1]);
	assign Result_Sltu = Result_S[`DATA_WIDTH];
	assign Result_Nor = ~(A | B);
	assign Result_Xor = A ^ B;
	assign Result_Mov = A;
	assign Result_Sll = B << A[4:0];
	assign Result_Srl = B >> A[4:0];
	assign Result_Sra = $signed(B) >>> A[4:0];
    assign Result = (ALUop == 4'b0000) ? Result_Add : //加
					(ALUop == 4'b0001) ? Result_And : //与
					(ALUop == 4'b0010) ? Result_Or : //或
					(ALUop == 4'b0011) ? Result_Xor : //异或
					(ALUop == 4'b0100) ? Result_Nor : //或非
					(ALUop == 4'b0101) ? Result_Sub : //减
					(ALUop == 4'b0110) ? Result_Slt : //有符号小于
					(ALUop == 4'b0111) ? Result_Sltu : //无符号小于
					(ALUop[3:1] == 3'b100 && (Signal ^ ALUop[0] == 0)) ? Result_Mov : //movn movz
					(ALUop == 4'b1010) ? Result_Sll : //逻辑左移
					(ALUop == 4'b1011) ? Result_Srl : //逻辑右移
					(ALUop == 4'b1100) ? Result_Sra : //算术右移
					32'b0; //默认为0
endmodule

//存取数转换器
module dataconverter(
	input [`DATA_WIDTH - 1:0] Instruction, //l s
	input [31:0] Address_in, //l s
	input [31:0] Data_rf2, //l s
	input [31:0] Data_in, //l
	output [31:0] Address_out, //l s
	output [31:0] Data_out, //l s
	output [3:0] Write_strb //s
);
	//地址对齐
	assign Address_out = {Address_in[31:2], 2'b00};
	wire [1:0] n = Address_in[1:0];

	//取字节 数据
	wire [31:0] Data_lb = (n == 2'b00) ? {{24{Data_in[7]}}, Data_in[7:0]} :
						(n == 2'b01) ? {{24{Data_in[15]}}, Data_in[15:8]} :
						(n == 2'b10) ? {{24{Data_in[23]}}, Data_in[23:16]} :
						(n == 2'b11) ? {{24{Data_in[31]}}, Data_in[31:24]} :
						32'b0;
	//取字节无符号扩展 数据
	wire [31:0] Data_lbu = (n == 2'b00) ? {{24'b0, Data_in[7:0]}} :
						(n == 2'b01) ? {{24'b0, Data_in[15:8]}} :
						(n == 2'b10) ? {{24'b0, Data_in[23:16]}} :
						(n == 2'b11) ? {{24'b0, Data_in[31:24]}} :
						32'b0;
	//取半字 数据
	wire [31:0] Data_lh = (n == 2'b00) ? {{16{Data_in[15]}}, Data_in[15:0]} :
						(n == 2'b10) ? {{16{Data_in[31]}}, Data_in[31:16]} :
						32'b0;
	//取半字无符号扩展 数据
	wire [31:0] Data_lhu = (n == 2'b00) ? {{16'b0, Data_in[15:0]}} :
						   (n == 2'b10) ? {{16'b0, Data_in[31:16]}} :
						   32'b0;
	//取字高位 数据
	wire [31:0] Data_lwl = (n == 2'b00) ? {Data_in[7:0], Data_rf2[23:0]} :
						   (n == 2'b01) ? {Data_in[15:0], Data_rf2[15:0]} :
						   (n == 2'b10) ? {Data_in[23:0], Data_rf2[7:0]} :
						   (n == 2'b11) ? Data_in :
						   32'b0;
	//取字低位 数据
	wire [31:0] Data_lwr = (n == 2'b00) ? Data_in :
							(n == 2'b01) ? {Data_rf2[31:24], Data_in[31:8]} :
							(n == 2'b10) ? {Data_rf2[31:16], Data_in[31:16]} :
							(n == 2'b11) ? {Data_rf2[31:8], Data_in[31:24]} :
							32'b0;
	//取数据选择
	wire [31:0] Data_load = (Instruction[28:26] == 3'b011) ? Data_in : //lw
							(Instruction[28:26] == 3'b000) ? Data_lb : //lb
							(Instruction[28:26] == 3'b001) ? Data_lh : //lh
							(Instruction[28:26] == 3'b100) ? Data_lbu : //lbu
							(Instruction[28:26] == 3'b101) ? Data_lhu : //lhu
							(Instruction[28:26] == 3'b010) ? Data_lwl : //lwl
							(Instruction[28:26] == 3'b110) ? Data_lwr : //lwr
							32'b0; //默认为0

	//存字节 数据及掩码
	wire [31:0] Data_sb = (n == 2'b00) ? {24'b0, Data_rf2[7:0]} :
						  (n == 2'b01) ? {16'b0, Data_rf2[7:0],8'b0} :
						  (n == 2'b10) ? {8'b0, Data_rf2[7:0],16'b0} :
						  (n == 2'b11) ? {Data_rf2[7:0],24'b0} :
						  32'b0;
	wire [3:0] Write_strb_sb = (n == 2'b00) ? 4'b0001 :
							(n == 2'b01) ? 4'b0010 :
							(n == 2'b10) ? 4'b0100 :
							(n == 2'b11) ? 4'b1000 :
							4'b0000;
	//存半字 数据及掩码
	wire [31:0] Data_sh = (n == 2'b00) ? {16'b0, Data_rf2[15:0]} :
						  (n == 2'b10) ? {Data_rf2[15:0],16'b0} :
						32'b0;
	wire [3:0] Write_strb_sh = (n == 2'b00) ? 4'b0011 :
							   (n == 2'b10) ? 4'b1100 :
							   4'b0000;
	//存字高位 数据及掩码
	wire [31:0] Data_swl = (n == 2'b00) ? {Data_rf2[23:0], Data_rf2[31:24]} :
					(n == 2'b01) ? {Data_rf2[15:0], Data_rf2[31:16]} :
					(n == 2'b10) ? {Data_rf2[7:0], Data_rf2[31:8]} :
					(n == 2'b11) ? Data_rf2 :
					32'b0;
	wire [3:0] Write_strb_swl = (n == 2'b00) ? 4'b0001 :
						(n == 2'b01) ? 4'b0011 :
						(n == 2'b10) ? 4'b0111 :
						(n == 2'b11) ? 4'b1111 :
						4'b0000;
	//存字低位 数据及掩码
	wire [31:0] Data_swr = (n == 2'b00) ? Data_rf2 :
						   (n == 2'b01) ? {Data_rf2[23:0], Data_rf2[31:24]} :
						   (n == 2'b10) ? {Data_rf2[15:0], Data_rf2[31:16]} :
						   (n == 2'b11) ? {Data_rf2[7:0], Data_rf2[31:8]} :
							32'b0;
	wire [3:0] Write_strb_swr = (n == 2'b00) ? 4'b1111 :
								(n == 2'b01) ? 4'b1110 :
								(n == 2'b10) ? 4'b1100 :
								(n == 2'b11) ? 4'b1000 :
								4'b0000;
	//存数据选择
	wire [31:0] Data_store = (Instruction[28:26] == 3'b010) ? Data_swl : //swl
							 (Instruction[28:26] == 3'b110) ? Data_swr : //swr
							 (Instruction[28:26] == 3'b000) ? Data_sb : //sb
							 (Instruction[28:26] == 3'b001) ? Data_sh : //sh
							 (Instruction[28:26] == 3'b011) ? Data_rf2 : //sw
							 32'b0; //默认为0
	//存掩码选择
	assign Write_strb = (Instruction[28:26] == 3'b010) ? Write_strb_swl : //swl
						(Instruction[28:26] == 3'b110) ? Write_strb_swr : //swr
						(Instruction[28:26] == 3'b000) ? Write_strb_sb : //sb
						(Instruction[28:26] == 3'b001) ? Write_strb_sh : //sh
						4'b1111; //默认为全写
	
	//数据选择
	assign Data_out = (Instruction[31:29] == 3'b100) ? Data_load : Data_store;

endmodule


//控制器
module controller(
	//输入指令字
	input [`DATA_WIDTH - 1:0] Instruction,
	input Zero,
	input Signal,
	//输出控制信号
	output [1:0] MemtoReg,
	output MemWrite,
	output [3:0] ALUop,
	output ALUSrc,
	output RegWrite,
	output RegDst,
	output [1:0] PCSrc,
	output ALUsa,
	output MemRead
);	
	//指令类型判断
	//R J I型指令
	wire R_type = (Instruction[31:26] == 6'b000000);
	wire J_type = (Instruction[31:26] == 6'b000010) || (Instruction[31:26] == 6'b000011);
	wire I_type = !R_type && !J_type;

	//使用sa移位
	assign ALUsa = R_type && (Instruction[5:2] == 4'b0000); //sll srl sra

	//分支信号
	wire Branch1 = (Instruction[31:27] == 5'b00010); //beq bne
	wire Branch2 = (Instruction[31:27] == 5'b00011); //bgtz blez
	wire Branch3 = (Instruction[31:26] == 6'b000001); //bgez bltz
	wire BranchSignal = Branch1 & (Zero ^ Instruction[26]) | Branch2 & ((Zero | Signal) ^ Instruction[26]) | Branch3 & (Signal);

	//PCSrc选择
	assign PCSrc = R_type && (Instruction[5:1] == 5'b00100) ? 2'b10 : //jr jalr
					J_type ? 2'b11 : //j jal
					BranchSignal ? 2'b01 : //beq bne bgtz blez bgez bltz
					2'b00; //默认为PC+4

	//MemtoReg选择
	wire jalr = R_type && (Instruction[5:0] == 6'b001001);
	assign MemtoReg = jalr || (Instruction[31:26] == 6'b000011) ? 2'b10 : //jalr jal
					  (Instruction[31:29] == 3'b100)? 2'b01 : //lw lwl lwr lb lbu lh lhu
					  2'b00; //默认为ALU_Result
	
	//RegDst选择
	assign RegDst = I_type ? 0 : 1;

	//内存读写使能
	assign MemRead = (Instruction[31:29] == 3'b100);//lw lwl lwr lb lbu lh lhu时读内存
	assign MemWrite = (Instruction[31:29] == 3'b101);//sw swl swr sb sh时写内存

	//寄存器写使能
	assign RegWrite = (I_type && (Instruction[31:29] == 3'b101 || Instruction[31:29] == 3'b000)) ? 0 : //立即数运算和存数
					  (Instruction[31:26] == 6'b000010 || R_type && Instruction[5:0] == 6'b001000) ? 0 : //j jr
					  (R_type && Instruction[5:1] == 5'b00101) ? (Signal ^ Instruction[0]) : 1; //movn movz

	//ALUSrc选择
	assign ALUSrc = (I_type && Instruction[31:29] != 3'b000) ? 1 : 0; //立即数运算和存取数

	//ALU控制信号
	//指令运算方式选择
	wire ALU_And, ALU_Or, ALU_Sub, ALU_Slt, ALU_Sltu, ALU_Nor, ALU_Xor, ALU_Movn, ALU_Movz, ALU_Sll, ALU_Srl, ALU_Sra, ALU_Bgez;
	//and andi or ori nor xor xori
	assign ALU_And = (R_type && (Instruction[5:0] == 6'b100100)) || (Instruction[31:26] == 6'b001100);
	assign ALU_Or = (R_type && (Instruction[5:0] == 6'b100101)) || (Instruction[31:26] == 6'b001101);
	assign ALU_Nor = R_type && (Instruction[5:0] == 6'b100111);
	assign ALU_Xor = (R_type && (Instruction[5:0] == 6'b100110)) || (Instruction[31:26] == 6'b001110);
	//slt slti sltu sltiu bgtz blez bltz bgez
	assign ALU_Bgez = Instruction[31:26] == 6'b000001 && Instruction[16];
	assign ALU_Slt = (R_type && (Instruction[5:0] == 6'b101010)) || (Instruction[31:26] == 6'b001010) 
		|| (Instruction[31:27] == 5'b00011) || (Instruction[31:26] == 6'b000001 && !Instruction[16]);
	assign ALU_Sltu = (R_type && (Instruction[5:0] == 6'b101011)) || (Instruction[31:26] == 6'b001011);
	//subu beq bne
	assign ALU_Sub = (R_type && (Instruction[5:0] == 6'b100011)) || (Instruction[31:27] == 5'b00010);
	//movn movz
	assign ALU_Movn = R_type && (Instruction[5:0] == 6'b001011);
	assign ALU_Movz = R_type && (Instruction[5:0] == 6'b001010);
	//sll srl sra
	assign ALU_Sll = R_type && (Instruction[5:0] == 6'b000000 || Instruction[5:0] == 6'b000100);
	assign ALU_Srl = R_type && (Instruction[5:0] == 6'b000010 || Instruction[5:0] == 6'b000110);
	assign ALU_Sra = R_type && (Instruction[5:0] == 6'b000011 || Instruction[5:0] == 6'b000111);
	
	//ALUop选择
	assign ALUop = (ALU_And) ? 4'b0001 : //and andi
				   (ALU_Or) ? 4'b0010 : //or ori
				   (ALU_Xor) ? 4'b0011 : //xor xori
				   (ALU_Nor) ? 4'b0100 : //nor
				   (ALU_Sub) ? 4'b0101 : //subu beq bne bgtz blez bltz
				   (ALU_Slt) ? 4'b0110 : //slt slti
				   (ALU_Sltu) ? 4'b0111 : //sltu sltiu
				   (ALU_Movn) ? 4'b1000 : //movn
				   (ALU_Movz) ? 4'b1001 : //movz
				   (ALU_Sll) ? 4'b1010 : //sll
				   (ALU_Srl) ? 4'b1011 : //srl
				   (ALU_Sra) ? 4'b1100 : //sra
				   (ALU_Bgez) ? 4'b1101 : //bgez
				   4'b0000; //默认为add
endmodule