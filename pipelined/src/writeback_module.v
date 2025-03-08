module writeback(
    input [63:0] mem_result,            
    input reg_write,                    
    input [4:0] rd_addr,                
    output [63:0] write_back_data,      
    output [4:0] write_back_addr,       
    output reg_write_back               
);
    
    assign write_back_data = mem_result; 
    assign write_back_addr = rd_addr;    
    assign reg_write_back = reg_write;   
endmodule
