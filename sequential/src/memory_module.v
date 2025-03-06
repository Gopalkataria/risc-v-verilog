module Memory (
    input clk,
    input reset,
    input [31:0] instr_addr,   
    input [31:0] data_addr,    
    input [31:0] write_data,   
    input MemRead,             
    input MemWrite,            
    output reg [31:0] instr,   
    output reg [31:0] read_data 
);
    
    reg [31:0] mem [0:16383];  
    
    
    parameter INSTR_BASE = 0;      
    parameter DATA_BASE = 8000;    

    
    integer i;
    initial begin
        
        for (i = 0; i < 16384; i = i + 1)
            mem[i] = 32'h0;
    end

    
    always @(posedge reset) begin
        if (reset) begin
            
            instr <= 32'h0;
            read_data <= 32'h0;
        end
    end

    
    always @(*) begin
        
        
        instr = (reset) ? 32'h0 : mem[INSTR_BASE + (instr_addr[13:2])];
    end

    
    always @(*) begin
        if (reset || !MemRead)
            read_data = 32'h0;
        else
            read_data = mem[DATA_BASE + (data_addr[13:2])];
    end

    
    always @(posedge clk) begin
        if (!reset && MemWrite)
            mem[DATA_BASE + (data_addr[13:2])] <= write_data;
    end
endmodule
