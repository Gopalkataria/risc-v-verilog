module WriteBack (
    input [31:0] ALU_result,
    input [31:0] mem_read_data,
    input MemtoReg,
    output [31:0] reg_write_data
);
    assign reg_write_data = MemtoReg ? mem_read_data : ALU_result;
endmodule