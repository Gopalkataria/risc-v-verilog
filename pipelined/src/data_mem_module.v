module DataMemory (
    input clk,
    input reset,
    input [31:0] data_addr,    
    input [31:0] write_data,   
    input MemRead,             
    input MemWrite,            
    output reg [31:0] read_data 
);
    reg [31:0] data_mem [0:8191];  
    integer i;
    initial begin
        for (i = 0; i < 8192; i = i + 1)
            data_mem[i] = 32'h0;
    end

    always @(*) begin
        if (reset || !MemRead)
            read_data = 32'h0;
        else
            read_data = data_mem[data_addr[13:2]];
    end

    always @(posedge clk) begin
        if (!reset && MemWrite)
            data_mem[data_addr[13:2]] <= write_data;
    end
endmodule