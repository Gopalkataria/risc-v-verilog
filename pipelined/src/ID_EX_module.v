module ID_EX(
    input clk,
    input reset,
    input ID_EX_Flush, // Control signal for pipeline stall or flush
    input [31:0] pc_in, // Program counter from Decode
    input [31:0] instr_in, // Instruction
    input [31:0] read_data1_in, // Register data 1 from Decode
    input [31:0] read_data2_in, // Register data 2 from Decode
    input [31:0] imm_in, // Immediate value from Decode
    input [4:0] rs1_in, rs2_in, rd_in, // Register addresses
    input [3:0] alu_ctrl_in, // ALU control signal from Decode
    input mem_read_in, mem_write_in, reg_write_in, mem_to_reg_in, alu_src_in, branch_in, jump_in, auipc_in, // Control signals
    output reg [31:0] pc_out,
    output reg [31:0] instr_out,
    output reg [31:0] read_data1_out,
    output reg [31:0] read_data2_out,
    output reg [31:0] imm_out,
    output reg [4:0] rs1_out, rs2_out, rd_out,
    output reg [3:0] alu_ctrl_out,
    output reg mem_read_out, mem_write_out, reg_write_out, mem_to_reg_out, alu_src_out, branch_out, jump_out, auipc_out
);

    always @(posedge clk or posedge reset) begin
        if (reset || ID_EX_Flush) begin
            // Flush pipeline stage when necessary
            pc_out <= 32'b0;
            instr_out <= 32'b0;
            read_data1_out <= 32'b0;
            read_data2_out <= 32'b0;
            imm_out <= 32'b0;
            rs1_out <= 5'b0;
            rs2_out <= 5'b0;
            rd_out <= 5'b0;
            alu_ctrl_out <= 4'b0;
            mem_read_out <= 0;
            mem_write_out <= 0;
            reg_write_out <= 0;
            mem_to_reg_out <= 0;
            alu_src_out <= 0;
            branch_out <= 0;
            jump_out <= 0;
            auipc_out <= 0;
        end
        else begin
            // Pass values from Decode stage to Execution stage
            pc_out <= pc_in;
            instr_out <= instr_in;
            read_data1_out <= read_data1_in;
            read_data2_out <= read_data2_in;
            imm_out <= imm_in;
            rs1_out <= rs1_in;
            rs2_out <= rs2_in;
            rd_out <= rd_in;
            alu_ctrl_out <= alu_ctrl_in;
            mem_read_out <= mem_read_in;
            mem_write_out <= mem_write_in;
            reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in;
            alu_src_out <= alu_src_in;
            branch_out <= branch_in;
            jump_out <= jump_in;
            auipc_out <= auipc_in;
        end
    end

endmodule
