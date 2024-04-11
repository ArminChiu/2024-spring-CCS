module top_module (
    input clk,
    input in,
    output reg out
);
    initial begin
    	out <=0;
    end
  // Write your code here
    reg in_before;
    always @(posedge clk) begin
        if(in == 1) begin
            if(in_before == 0) begin
                out <= 1;
                in_before <= 1;
            end else begin
                out <= 0;
            end
        end else begin
            in_before <= 0;
            out <= 0;
        end
    end
endmodule
