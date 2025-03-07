module EX_MEM(
    input clk, reset,
    input [31:0] ALU_result_in, reg2_in,
    input [4:0] rd_in,
    input mem_read_in, mem_write_in, reg_write_in, mem_to_reg_in,
    output reg [31:0] ALU_result_out, reg2_out,
    output reg [4:0] rd_out,
    output reg mem_read_out, mem_write_out, reg_write_out, mem_to_reg_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ALU_result_out <= 0;
            reg2_out <= 0;
            rd_out <= 0;
            mem_read_out <= 0;
            mem_write_out <= 0;
            reg_write_out <= 0;
            mem_to_reg_out <= 0;
        end else begin
            ALU_result_out <= ALU_result_in;
            reg2_out <= reg2_in;
            rd_out <= rd_in;
            mem_read_out <= mem_read_in;
            mem_write_out <= mem_write_in;
            reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in;
        end
    end
endmodule