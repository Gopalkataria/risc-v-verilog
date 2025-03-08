module decode(
    input clk,
    input rst,
    input [31:0] instruction,
    input [63:0] pc,
    input instruction_valid,
    input reg_write_back,
    input [4:0] write_back_addr,
    input [63:0] write_back_data,
    output [63:0] rs1_data,
    output [63:0] rs2_data,
    output [63:0] imm,
    output [63:0] branch_target,
    output mem_read,
    output mem_write,
    output reg_write,
    output [4:0] rs1_addr,
    output [4:0] rs2_addr,
    output [4:0] rd_addr,
    output [2:0] funct3,
    output [6:0] funct7,
    output [6:0] opcode,
    output alu_src,
    output branch,
    output jump,
    output mem_to_reg,
    output [1:0] alu_op
);
    
    assign opcode = instruction[6:0];
    assign rs1_addr = instruction[19:15];
    assign rs2_addr = instruction[24:20];
    assign rd_addr = instruction[11:7];
    assign funct3 = instruction[14:12];
    assign funct7 = instruction[31:25];

    
    register_file reg_file(
        .clk(clk),
        .rst(rst),
        .rs1_addr(rs1_addr),
        .rs2_addr(rs2_addr),
        .rd_addr(write_back_addr),
        .rd_data(write_back_data),
        .reg_write(reg_write_back),
        .rs1_data(rs1_data),
        .rs2_data(rs2_data)
    );

    
    wire [63:0] imm_i = {{52{instruction[31]}}, instruction[31:20]};
    wire [63:0] imm_s = {{52{instruction[31]}}, instruction[31:25], instruction[11:7]};
    wire [63:0] imm_b = {{51{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
    wire [63:0] imm_u = {{32{instruction[31]}}, instruction[31:12], 12'b0}; 
    wire [63:0] imm_j = {{43{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};

    
    reg [63:0] imm_reg;
    always @(*) begin
        case (opcode)
            7'b0000011: imm_reg = imm_i; 
            7'b0010011: imm_reg = imm_i; 
            7'b0100011: imm_reg = imm_s; 
            7'b1100011: imm_reg = imm_b; 
            7'b0110111: imm_reg = imm_u; 
            7'b0010111: imm_reg = imm_u; 
            7'b1101111: imm_reg = imm_j; 
            7'b1100111: imm_reg = imm_i; 
            default:    imm_reg = 64'b0;
        endcase
    end
    assign imm = imm_reg;

    
    assign branch_target = pc + imm;

    
    assign mem_read = (opcode == 7'b0000011) && instruction_valid;
    assign mem_write = (opcode == 7'b0100011) && instruction_valid;
    
    
    assign reg_write = ((opcode == 7'b0110011) || 
                        (opcode == 7'b0010011) || 
                        (opcode == 7'b0000011) || 
                        (opcode == 7'b1101111) || 
                        (opcode == 7'b1100111) || 
                        (opcode == 7'b0110111) || 
                        (opcode == 7'b0010111))   
                        && instruction_valid;
    
    
    assign alu_src = (opcode == 7'b0010011) || 
                     (opcode == 7'b0100011) || 
                     (opcode == 7'b0000011) || 
                     (opcode == 7'b1100111) || 
                     (opcode == 7'b0110111) || 
                     (opcode == 7'b0010111);   
    
    
    assign branch = (opcode == 7'b1100011) && instruction_valid; 
    assign jump = ((opcode == 7'b1101111) || 
                  (opcode == 7'b1100111))    
                  && instruction_valid;
    
    
    assign mem_to_reg = (opcode == 7'b0000011) && instruction_valid;
    
    
    
    assign alu_op = (opcode == 7'b0110011) ? 2'b10 : 
                   (opcode == 7'b0010011) ? 2'b11 :  
                   (opcode == 7'b1100011) ? 2'b01 :  
                   (opcode == 7'b1100111) ? 2'b00 :  
                   2'b00;                            

endmodule


module id_ex_register(
    input clk,                          
    input rst,                          
    input stall,                        
    input flush,                        
    input [63:0] pc_in,                 
    input [63:0] rs1_data_in,           
    input [63:0] rs2_data_in,           
    input [63:0] imm_in,                
    input [63:0] branch_target_in,      
    input mem_read_in,                  
    input mem_write_in,                 
    input reg_write_in,                 
    input [4:0] rs1_addr_in,            
    input [4:0] rs2_addr_in,            
    input [4:0] rd_addr_in,             
    input [2:0] funct3_in,              
    input [6:0] funct7_in,              
    input [6:0] opcode_in,              
    input alu_src_in,                   
    input branch_in,                    
    input jump_in,                      
    input mem_to_reg_in,                
    output reg [63:0] pc_out,           
    output reg [63:0] rs1_data_out,     
    output reg [63:0] rs2_data_out,     
    output reg [63:0] imm_out,          
    output reg [63:0] branch_target_out, 
    output reg mem_read_out,            
    output reg mem_write_out,           
    output reg reg_write_out,           
    output reg [4:0] rs1_addr_out,      
    output reg [4:0] rs2_addr_out,      
    output reg [4:0] rd_addr_out,       
    output reg [2:0] funct3_out,        
    output reg [6:0] funct7_out,        
    output reg [6:0] opcode_out,        
    output reg alu_src_out,             
    output reg branch_out,              
    output reg jump_out,                
    output reg mem_to_reg_out   
);
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            
            pc_out <= 64'b0;
            rs1_data_out <= 64'b0;
            rs2_data_out <= 64'b0;
            imm_out <= 64'b0;
            branch_target_out <= 64'b0;
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            reg_write_out <= 1'b0;
            rs1_addr_out <= 5'b0;
            rs2_addr_out <= 5'b0;
            rd_addr_out <= 5'b0;
            funct3_out <= 3'b0;
            funct7_out <= 7'b0;
            opcode_out <= 7'b0;
            alu_src_out <= 1'b0;
            branch_out <= 1'b0;
            jump_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
        end else if (flush) begin
            
            pc_out <= 64'b0;
            rs1_data_out <= 64'b0;
            rs2_data_out <= 64'b0;
            imm_out <= 64'b0;
            branch_target_out <= 64'b0;
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            reg_write_out <= 1'b0;
            rs1_addr_out <= 5'b0;
            rs2_addr_out <= 5'b0;
            rd_addr_out <= 5'b0;
            funct3_out <= 3'b0;
            funct7_out <= 7'b0;
            opcode_out <= 7'b0;
            alu_src_out <= 1'b0;
            branch_out <= 1'b0;
            jump_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
        end else if (stall) begin
            
            pc_out <= pc_out;
            rs1_data_out <= rs1_data_out;
            rs2_data_out <= rs2_data_out;
            imm_out <= imm_out;
            branch_target_out <= branch_target_out;
            mem_read_out <= mem_read_out;
            mem_write_out <= mem_write_out;
            reg_write_out <= reg_write_out;
            rs1_addr_out <= rs1_addr_out;
            rs2_addr_out <= rs2_addr_out;
            rd_addr_out <= rd_addr_out;
            funct3_out <= funct3_out;
            funct7_out <= funct7_out;
            opcode_out <= opcode_out;
            alu_src_out <= alu_src_out;
            branch_out <= branch_out;
            jump_out <= jump_out;
            mem_to_reg_out <= mem_to_reg_out;
        end else begin
            
            pc_out <= pc_in;
            rs1_data_out <= rs1_data_in;
            rs2_data_out <= rs2_data_in;
            imm_out <= imm_in;
            branch_target_out <= branch_target_in;
            mem_read_out <= mem_read_in;
            mem_write_out <= mem_write_in;
            reg_write_out <= reg_write_in;
            rs1_addr_out <= rs1_addr_in;
            rs2_addr_out <= rs2_addr_in;
            rd_addr_out <= rd_addr_in;
            funct3_out <= funct3_in;
            funct7_out <= funct7_in;
            opcode_out <= opcode_in;
            alu_src_out <= alu_src_in;
            branch_out <= branch_in;
            jump_out <= jump_in;
            mem_to_reg_out <= mem_to_reg_in;
        end
    end
endmodule
