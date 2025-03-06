module ID_EX (
    input clk, reset,
    input [31:0] pc_in, reg1_in, reg2_in, imm_in,
    input [4:0] rs1_in, rs2_in, rd_in,
    input [3:0] alu_ctrl_in,
    input mem_read_in, mem_write_in, reg_write_in, mem_to_reg_in,
    output reg [31:0] pc_out, reg1_out, reg2_out, imm_out,
    output reg [4:0] rs1_out, rs2_out, rd_out,
    output reg [3:0] alu_ctrl_out,
    output reg mem_read_out, mem_write_out, reg_write_out, mem_to_reg_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_out <= 0;
            reg1_out <= 0;
            reg2_out <= 0;
            imm_out <= 0;
            rs1_out <= 0;
            rs2_out <= 0;
            rd_out <= 0;
            alu_ctrl_out <= 0;
            mem_read_out <= 0;
            mem_write_out <= 0;
            reg_write_out <= 0;
            mem_to_reg_out <= 0;
        end else begin
            pc_out <= pc_in;
            reg1_out <= reg1_in;
            reg2_out <= reg2_in;
            imm_out <= imm_in;
            rs1_out <= rs1_in;
            rs2_out <= rs2_in;
            rd_out <= rd_in;
            alu_ctrl_out <= alu_ctrl_in;
            mem_read_out <= mem_read_in;
            mem_write_out <= mem_write_in;
            reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in;
        end
    end
endmodule
