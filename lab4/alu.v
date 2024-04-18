`define DATA_WIDTH 32
module top_module(
  	input [`DATA_WIDTH - 1:0] A,
	input [`DATA_WIDTH - 1:0] B,
	input [2:0] ALUop,
	output Overflow,
	output CarryOut,
	output Zero,
	output [`DATA_WIDTH - 1:0] Result
);

    //加减法
    wire [`DATA_WIDTH:0] Result_Add;
    wire [`DATA_WIDTH:0] Result_Sub;
    assign Result_Add = A + B;
    assign Result_Sub = A + ~B + 1;

    //溢出
    wire Overflow_Add;
    wire Overflow_Sub;

    assign Overflow_Add = (A[`DATA_WIDTH - 1] == B[`DATA_WIDTH - 1]) && (A[`DATA_WIDTH - 1] != Result_Add[`DATA_WIDTH - 1]);
    assign Overflow_Sub = (A[`DATA_WIDTH - 1] != B[`DATA_WIDTH - 1]) && (A[`DATA_WIDTH - 1] != Result_Sub[`DATA_WIDTH - 1]);
    assign Overflow = (ALUop[2] == 0) ? Overflow_Add : Overflow_Sub;
    
    //进借位
    wire CarryOut_Add;
    wire CarryOut_Sub;
    
    assign CarryOut_Add = Result_Add[`DATA_WIDTH];
    assign CarryOut_Sub = Result_Sub[`DATA_WIDTH];
    assign CarryOut = (ALUop[2] == 0) ? CarryOut_Add : CarryOut_Sub;

    //零
    assign Zero = (Result == 0);
    
    //根据操作码对Result赋值
    reg [`DATA_WIDTH - 1:0] Result;

    always @(*) begin
        case(ALUop)
            3'b000: Result = A & B;
            3'b001: Result = A | B;
            3'b010: Result = Result_Add;
            3'b110: Result = Result_Sub;
            3'b111: Result = (Result_Sub[`DATA_WIDTH - 1] & ~Overflow_Sub) | (Overflow_Sub & ~Result_Sub[`DATA_WIDTH - 1]);
        endcase
    end
    
endmodule