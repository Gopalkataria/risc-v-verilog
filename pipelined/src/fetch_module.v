module Fetch(
    input clk,
    input reset,
    input PCWrite, // Controls whether PC updates or stalls
    input [1:0] PCSrc, // Determines next PC source
    input [31:0] branch_target, // Target for branching
    input [31:0] alu_result, // ALU result for jump and branch calculations
    input Instr_Flush, // Control signal to flush IF/ID register
    input IF_ID_Write, // Control signal to allow IF/ID update
    output reg [31:0] next_PC, // Next PC value
    output reg [31:0] current_PC, // Current PC value
    output [31:0] instr // Fetched instruction
);

    wire [31:0] instr_mem_out;
    
    // Use existing Instruction Memory module
    InstructionMemory instr_mem(
        .instr_addr(current_PC),
        .instr(instr_mem_out)
    );
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_PC <= 32'h0;
        end else if (PCWrite) begin
            case (PCSrc)
                2'b00: next_PC = current_PC + 4; // Normal sequential execution
                2'b01: next_PC = branch_target; // Branch target address
                2'b10: next_PC = alu_result; // Jump target address
                default: next_PC = current_PC + 4;
            endcase
            current_PC <= next_PC;
        end
    end
    
    assign instr = Instr_Flush ? 32'h00000013 : instr_mem_out; // NOP if flushed

endmodule