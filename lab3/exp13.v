module top_module (
    input clk,
    input [7:0] d,
  	output reg[7:0] q
);

  //Add code below
    always @(posedge clk) begin
        q <= d;
    end
endmodule