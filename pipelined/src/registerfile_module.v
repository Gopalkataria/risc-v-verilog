module RegisterFile(
    input clk,
    input rst,
    input RegWrite,
    input [4:0] rs1, rs2, rd,
    input [31:0] write_data,
    output [31:0] read_data1,
    output [31:0] read_data2
);
    reg [31:0] registers [0:31];

    // Initialize registers to 0
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            registers[i] = 32'h0;
    end

    // Read registers asynchronously
    assign read_data1 = (rs1 != 0) ? registers[rs1] : 32'h0;
    assign read_data2 = (rs2 != 0) ? registers[rs2] : 32'h0;

    // Write to register file on the rising edge of the clock
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1)
                registers[i] <= 32'h0;
        end
        else if (RegWrite && rd != 0) begin
            registers[rd] <= write_data;
        end
    end
endmodule
