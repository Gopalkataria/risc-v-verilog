module fetch(
    input clk,                          
    input rst,                          
    input stall,                        
    input [63:0] branch_target,         
    input branch_taken,                 
    output reg [63:0] pc,               
    output wire [31:0] instruction,      
    output reg instruction_valid        
);
    
    instruction_memory imem(
        .pc(pc),                        
        .instruction(instruction)       
    );

    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc <= 64'b0;                
            instruction_valid <= 1'b0;  
        end else if (stall) begin
            
            pc <= pc;
            instruction_valid <= 1'b0;  
        end else if (branch_taken) begin
            
            pc <= branch_target;
            instruction_valid <= 1'b1;  
        end else begin
            
            pc <= pc + 64'd4;
            instruction_valid <= 1'b1; 
        end
    end
endmodule


module IF_ID (
    input clk,                          
    input rst,                          
    input stall,                        
    input flush,                        
    input [63:0] pc_in,                 
    input [31:0] instruction_in,        
    input instruction_valid_in,         
    output reg [63:0] pc_out,           
    output reg [31:0] instruction_out,  
    output reg instruction_valid_out    
);
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            
            pc_out <= 64'b0;
            instruction_out <= 32'b0;
            instruction_valid_out <= 1'b0;
        end else if (flush) begin
            
            pc_out <= 64'b0;
            instruction_out <= 32'b0;
            instruction_valid_out <= 1'b0;
        end else if (stall) begin
            
            pc_out <= pc_out;
            instruction_out <= instruction_out;
            instruction_valid_out <= instruction_valid_out;
        end else begin
            
            pc_out <= pc_in;
            instruction_out <= instruction_in;
            instruction_valid_out <= instruction_valid_in;
        end
    end
endmodule
