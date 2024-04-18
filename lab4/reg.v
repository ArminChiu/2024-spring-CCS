`timescale 10 ns / 1 ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

module top_module(
	input clk,
	input rst,
	input [`ADDR_WIDTH - 1:0] waddr,
	input [`ADDR_WIDTH - 1:0] raddr1,
	input [`ADDR_WIDTH - 1:0] raddr2,
	input wen,
	input [`DATA_WIDTH - 1:0] wdata,
	output [`DATA_WIDTH - 1:0] rdata1,
	output [`DATA_WIDTH - 1:0] rdata2
);
    
    //寄存器堆
	reg [`DATA_WIDTH - 1:0] mem [0:2**`ADDR_WIDTH - 1];

    //初始化
    genvar i;
    generate
        for(i =0;i<1;i=i+1) begin
            always @(negedge rst) begin
                if(!rst) begin
                    mem[i]<=0;
                end
            end
        end
    endgenerate

    //异步读
	assign rdata1 = mem[raddr1];
	assign rdata2 = mem[raddr2];

    //同步写
    always @(posedge clk or posedge rst) begin
        begin
            if(wen) begin
                if(waddr != 0) begin
                    mem[waddr] <= wdata;
                end
            end
        end
    end
 
endmodule