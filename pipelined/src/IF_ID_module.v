module IF_ID(
    input clk,
    input reset,
    input IF_ID_Write,  // Control signal to stall pipeline
    input Instr_Flush,  // Control signal to flush instruction
    input [31:0] pc_in, instr_in,
    output reg [31:0] pc_out, instr_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_out <= 32'h0;
            instr_out <= 32'h0;
        end 
        else if (Instr_Flush) begin
            pc_out <= 32'h0;
            instr_out <= 32'h00000013; // NOP (ADDI x0, x0, 0)
        end
        else if (IF_ID_Write) begin
            pc_out <= pc_in;
            instr_out <= instr_in;
        end
    end

endmodule
