`timescale 10 ns / 1 ns

`include "test/define.v"
`include "test/ideal_mem.v"
module testbench ();
	reg mips_cpu_clk, mips_cpu_reset;
	
	
	localparam		MEM_ADDR_WIDTH = 12;
	//MIPS CPU ports to ideal memory
	wire [31:0]		mips_mem_addr;
	wire			MemWrite;
	wire [31:0]		mips_mem_wdata;
	wire			MemRead;
	wire [3:0]		mips_mem_wstrb;
	wire [31:0]		mips_mem_rdata;

	wire [31:0]		PC;
	wire [31:0]		Instruction;
	
	//Ideal memory ports
	wire [MEM_ADDR_WIDTH - 3:0]	Waddr;
	wire [MEM_ADDR_WIDTH - 3:0]	Raddr;

	wire			Wren;
	wire [31:0]		Wdata;
	wire [3:0]		Wstrb;
	wire			Rden;
	wire [31:0]		Rdata;

	//Shadow stack
	wire            mips_sstk_notmatch;
	wire [31:0]     Ra_val;
	
	top_module u_mips_cpu(
		.clk(mips_cpu_clk),
		.rst(mips_cpu_reset),
		.PC(PC),
		.Instruction(Instruction),
		.Address(mips_mem_addr),
		.MemWrite(MemWrite),
		.Write_data(mips_mem_wdata),
		.Write_strb(mips_mem_wstrb),
		.Read_data(mips_mem_rdata),
		.MemRead(MemRead)
	);
	assign Wren = MemWrite;
	assign Wdata = {32{MemWrite}} & mips_mem_wdata;
	assign Wstrb = {4{MemWrite}} & mips_mem_wstrb;
	assign Waddr = {MEM_ADDR_WIDTH-2{MemWrite}} & mips_mem_addr[MEM_ADDR_WIDTH - 1:2] ;

	assign mips_mem_rd = MemRead & (~mips_cpu_reset);
	assign Rden = mips_mem_rd;
	assign mips_mem_rdata = {32{mips_mem_rd}} & Rdata;
	assign Raddr = {MEM_ADDR_WIDTH-2{mips_mem_rd}} & mips_mem_addr[MEM_ADDR_WIDTH - 1:2] ;

	ideal_mem		# (
	  .ADDR_WIDTH	(MEM_ADDR_WIDTH)
	) u_ideal_mem (
	  .clk			(mips_cpu_clk),
	  
	  .Waddr		(Waddr),
	  .Raddr1		(PC[MEM_ADDR_WIDTH - 1:2]),
	  .Raddr2		(Raddr),

	  .Wren			(Wren),
	  .Rden1		(1'b1),
	  .Rden2		(Rden),

	  .Wdata		(Wdata),
	  .Wstrb		(Wstrb),
	  .Rdata1		(Instruction),
	  .Rdata2		(Rdata)
  	);

	initial begin
		mips_cpu_clk = 1'b0;
		mips_cpu_reset = 1'b1;
		#3 
		mips_cpu_reset = 1'b0;
		#(`CYCLE) 
		$writememh("mem_dump.hex", u_ideal_mem.mem);
		$finish;
	end
	initial begin
        $dumpfile("waveform/out.vcd");
        $dumpvars;
        //$monitor("%1d\t%b\t%b\t%b",$time,operand0,operand1,result);
   end 
//	initial #10000 $finish;

	always begin
		#1 mips_cpu_clk = ~mips_cpu_clk;
	end

	//-----------------------------------------------------------------
	// code below from ict
	`define RST u_mips_cpu.rst

	`define RF_WEN   u_mips_cpu.RF_wen
	`define RF_WADDR u_mips_cpu.RF_waddr
	`define RF_WDATA u_mips_cpu.RF_wdata

	`define MEM_WEN   u_mips_cpu.MemWrite
	`define MEM_ADDR  u_mips_cpu.Address
	`define MEM_WSTRB u_mips_cpu.Write_strb
	`define MEM_WDATA u_mips_cpu.Write_data
	`define MEM_READ  u_mips_cpu.MemRead

	`define PC u_mips_cpu.PC
	integer testcase,trace_file, type, PC_ref, new_PC_ref;
	integer rf_waddr_ref, rf_wdata_ref, rf_bit_cmp_ref;
	integer mem_addr_ref, mem_wdata_ref, mem_bit_cmp_ref;
	reg [3:0] mem_wstrb_ref;
	reg mem_read_ref;
	integer ret;
	reg [20:0] testcase_name;

	
	initial begin
        $readmemh(`INST_MEM,u_ideal_mem.mem);
    end

endmodule // Test Bench;
