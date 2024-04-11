module tb();
wire [2:0]out;//必要输出信号 	
	//信号定义
    reg clk;
	//信号生成
    initial begin
        clk = 1'b0;
        forever begin
        #5 clk = ~clk;
        end
    end
	//模块例化
    dut dut0(.clk(clk),.out(out));
endmodule

module dut(input clk, output reg [2:0]out);
  //测试模块
  always @(posedge clk)
    out <= out + 1'b1;  
endmodule