module memory(
    input clk,                          
    input rst,                          
    input [63:0] alu_result,           
    input [63:0] mem_address,          
    input [63:0] mem_write_data,       
    input branch_taken,                
    input [63:0] jump_target,          
    input reg_write,                   
    input [4:0] rd_addr,               
    input [2:0] funct3,                
    input [6:0] funct7,                
    input mem_read,                    
    input mem_write,                   
    input mem_to_reg,                  
    output wire [63:0] mem_read_data,  
    output reg [63:0] mem_result,      
    output reg reg_write_out,          
    output reg [4:0] rd_addr_out,      
    output reg mem_to_reg_out          
);
    
    data_memory dmem(
        .clk(clk),
        .address(mem_address),
        .write_data(mem_write_data),
        .mem_read(mem_read),           
        .mem_write(mem_write),         
        .read_data(mem_read_data)      
    );

   
    always @(*) begin
        if (mem_to_reg) begin
           
            case (funct3)
                3'b000: mem_result = {{56{mem_read_data[7]}}, mem_read_data[7:0]};
                3'b001: mem_result = {{48{mem_read_data[15]}}, mem_read_data[15:0]};
                3'b010: mem_result = {{32{mem_read_data[31]}}, mem_read_data[31:0]};
                3'b011: mem_result = mem_read_data;
                3'b100: mem_result = {56'b0, mem_read_data[7:0]};
                3'b101: mem_result = {48'b0, mem_read_data[15:0]};
                3'b110: mem_result = {32'b0, mem_read_data[31:0]};
                default: mem_result = 64'b0;
            endcase
        end else begin
           
            mem_result = alu_result;
        end
    end

   
    always @(*) begin
        reg_write_out = reg_write;     
        rd_addr_out = rd_addr;        
        mem_to_reg_out = mem_to_reg;  
    end
endmodule




module mem_wb_register(
    input clk,                          
    input rst,                          
    input stall,                        
    input flush, 
    input mem_to_reg_in,
    input mem_to_reg_out, 
    input mem_to_reg ,                       
    input [63:0] mem_result_in,         
    input reg_write_in,                 
    input [4:0] rd_addr_in,             
    output reg [63:0] mem_result_out,   
    output reg reg_write_out,           
    output reg [4:0] rd_addr_out        
);
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            
            mem_result_out <= 64'b0;
            reg_write_out <= 1'b0;
            rd_addr_out <= 5'b0;
        end else if (flush) begin
            
            mem_result_out <= 64'b0;
            reg_write_out <= 1'b0;
            rd_addr_out <= 5'b0;
        end else if (stall) begin
            
            mem_result_out <= mem_result_out;
            reg_write_out <= reg_write_out;
            rd_addr_out <= rd_addr_out;
        end else begin
            
            mem_result_out <= mem_result_in;
            reg_write_out <= reg_write_in;
            rd_addr_out <= rd_addr_in;
        end
    end
endmodule

