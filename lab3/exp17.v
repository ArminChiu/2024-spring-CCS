module top_module(
input 		  clk,
input		  wr_en,
input 	[2:0]  wr_addr,
input	[15:0] wr_data,
input	[2:0]  rd_addr,
output	[15:0] rd_data);
  
    reg	[15:0] mem[0:7];
	initial
	begin
      $readmemh("testcase/memfile.data",mem);
	end

   
    
    //Write Code Here
    assign rd_data = mem[rd_addr];

    always @(posedge clk) begin
        if(wr_en) begin
            mem[wr_addr] <= wr_data;
        end
    end
endmodule
