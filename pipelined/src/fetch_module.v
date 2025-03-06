module Fetch (
    input clk,
    input reset,
    input PCWrite,
    input [1:0] PCSrc,
    input [31:0] branch_target,
    input [31:0] alu_result,
    output reg [31:0] next_PC,
    input [31:0] current_PC
);
    
    always @(*) begin
        case (PCSrc)
            2'b00: next_PC = current_PC + 4;        
            2'b01: next_PC = branch_target;         
            2'b10: next_PC = alu_result;            
            default: next_PC = current_PC + 4;
        endcase
    end
endmodule