module Memory (
    input clk,
    input [31:0] address,
    input [31:0] write_data,
    input MemRead,
    input MemWrite,
    output [31:0] read_data
);
    reg [31:0] data_mem [0:1023];  // Example: 4KB memory

    // Combinational read
    assign read_data = MemRead ? data_mem[address >> 2] : 0;

    // Synchronous write
    always @(posedge clk) begin
        if (MemWrite)
            data_mem[address >> 2] <= write_data;
    end
endmodule