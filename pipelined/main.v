module riscv_processor(
    input clk,                          
    input rst                           
);
    
    // Fetch stage signals
    wire [63:0] if_pc;                  
    wire [31:0] if_instruction;         
    wire if_instruction_valid;          
    
    // IF/ID pipeline register signals
    wire [63:0] if_id_pc;               
    wire [31:0] if_id_instruction;      
    wire if_id_instruction_valid;       
    
    // Decode stage signals
    wire [63:0] id_rs1_data, id_rs2_data, id_imm, id_branch_target;
    wire id_alu_src, id_branch, id_jump, id_mem_to_reg;
    wire [1:0] id_alu_op;
    wire id_mem_read, id_mem_write, id_reg_write;
    wire [4:0] id_rs1_addr, id_rs2_addr, id_rd_addr;
    wire [2:0] id_funct3;
    wire [6:0] id_funct7, id_opcode;
    
    // ID/EX pipeline register signals
    wire [63:0] id_ex_pc;               
    wire [63:0] id_ex_rs1_data;         
    wire [63:0] id_ex_rs2_data;         
    wire [63:0] id_ex_imm;              
    wire [63:0] id_ex_branch_target;    
    wire id_ex_mem_read;                
    wire id_ex_mem_write;               
    wire id_ex_reg_write;               
    wire [4:0] id_ex_rs1_addr;          
    wire [4:0] id_ex_rs2_addr;          
    wire [4:0] id_ex_rd_addr;           
    wire [2:0] id_ex_funct3;            
    wire [6:0] id_ex_funct7;
    wire [6:0] id_ex_opcode;            
    wire id_ex_alu_src, id_ex_branch, id_ex_jump, id_ex_mem_to_reg;
    wire [1:0] id_ex_alu_op;        
    
    // Execute stage signals
    wire [63:0] ex_alu_result;          
    wire [63:0] ex_mem_address;         
    wire [63:0] ex_mem_write_data;      
    wire ex_branch_taken;               
    wire [63:0] ex_jump_target;       
    wire ex_reg_write;                  
    wire [4:0] ex_rd_addr;              
    wire ex_mem_read;                  
    wire ex_mem_write;                 
    wire [2:0] ex_funct3;              
    wire [6:0] ex_funct7;              
    wire ex_mem_to_reg;                
    wire [63:0] ex_branch_target_out;   // Fixed: 64 bits
    wire [4:0] ex_rd_addr_out;          // Fixed: 5 bits
    wire [2:0] ex_funct3_out;           // Fixed: 3 bits
    wire [6:0] ex_funct7_out;           // Fixed: 7 bits
    
    // EX/MEM pipeline register signals
    wire [63:0] ex_mem_alu_result;      
    wire [63:0] ex_mem_mem_address;     
    wire [63:0] ex_mem_mem_write_data;  
    wire ex_mem_branch_taken;           
    wire [63:0] ex_mem_jump_target;   
    wire ex_mem_reg_write;              
    wire [4:0] ex_mem_rd_addr;          
    wire [2:0] ex_mem_funct3;          
    wire [6:0] ex_mem_funct7;          
    wire ex_mem_mem_read;              
    wire ex_mem_mem_write;             
    wire ex_mem_mem_to_reg;            
    wire [63:0] ex_mem_branch_target;   // Fixed: 64 bits
    
    // Memory stage signals
    wire [63:0] mem_read_data;
    wire [63:0] mem_result;             
    wire mem_reg_write;                 
    wire [4:0] mem_rd_addr;             
    wire mem_mem_to_reg;               
    
    // MEM/WB pipeline register signals
    wire [63:0] mem_wb_mem_result;      
    wire mem_wb_reg_write;              
    wire [4:0] mem_wb_rd_addr;          
    wire mem_wb_mem_to_reg;            
    
    // Writeback stage signals
    wire [63:0] write_back_data;        
    wire [4:0] write_back_addr;         
    wire write_back_enable;             
    
    // Hazard detection and control signals
    wire stall;                         
    wire flush;                         
    
    // Branch control signals
    wire branch_taken;                  
    wire [63:0] branch_target;          
    
    // Assign branch control signals
    assign branch_taken = ex_mem_branch_taken;
    assign branch_target = ex_mem_branch_target;
    
    // Assign flush signal
    assign flush = branch_taken;
    
    // Hazard Detection Unit
    hazard_detection_unit hdu(
        .id_ex_rs1_addr(id_ex_rs1_addr),
        .id_ex_rs2_addr(id_ex_rs2_addr),
        .ex_mem_rd_addr(ex_mem_rd_addr),
        .mem_wb_rd_addr(mem_wb_rd_addr),
        .id_ex_mem_read(id_ex_mem_read),
        .stall(stall)
    );
    
    // Fetch stage
    fetch fetch_stage(
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .branch_target(branch_target),
        .branch_taken(branch_taken),
        .pc(if_pc),
        .instruction(if_instruction),
        .instruction_valid(if_instruction_valid)
    );
    
    // IF/ID pipeline register
    IF_ID if_id_register(
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .flush(flush),
        .pc_in(if_pc),
        .instruction_in(if_instruction),
        .instruction_valid_in(if_instruction_valid),
        .pc_out(if_id_pc),
        .instruction_out(if_id_instruction),
        .instruction_valid_out(if_id_instruction_valid)
    );
    
    // Decode stage
    decode decode_stage(
        .clk(clk),
        .rst(rst),
        .instruction(if_id_instruction),
        .pc(if_id_pc),
        .instruction_valid(if_id_instruction_valid),
        .reg_write_back(write_back_enable),
        .write_back_addr(write_back_addr),
        .write_back_data(write_back_data),
        .rs1_data(id_rs1_data),              
        .rs2_data(id_rs2_data),              
        .imm(id_imm),                        
        .branch_target(id_branch_target),    
        .mem_read(id_mem_read),              
        .mem_write(id_mem_write),            
        .reg_write(id_reg_write),            
        .rs1_addr(id_rs1_addr),              
        .rs2_addr(id_rs2_addr),              
        .rd_addr(id_rd_addr),                
        .funct3(id_funct3),                  
        .funct7(id_funct7),
        .opcode(id_opcode),
        .alu_src(id_alu_src),
        .branch(id_branch),
        .jump(id_jump),
        .mem_to_reg(id_mem_to_reg),
        .alu_op(id_alu_op)            
    );
    
    // ID/EX pipeline register
    id_ex_register id_ex_register(
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .flush(flush),
        .pc_in(if_id_pc),
        .rs1_data_in(id_rs1_data),           
        .rs2_data_in(id_rs2_data),           
        .imm_in(id_imm),                     
        .branch_target_in(id_branch_target), 
        .mem_read_in(id_mem_read),           
        .mem_write_in(id_mem_write),         
        .reg_write_in(id_reg_write),         
        .rs1_addr_in(id_rs1_addr),           
        .rs2_addr_in(id_rs2_addr),           
        .rd_addr_in(id_rd_addr),             
        .funct3_in(id_funct3),               
        .funct7_in(id_funct7),
        .opcode_in(id_opcode),
        .alu_src_in(id_alu_src),
        .branch_in(id_branch),
        .jump_in(id_jump),
        .mem_to_reg_in(id_mem_to_reg),
        .pc_out(id_ex_pc),
        .rs1_data_out(id_ex_rs1_data),
        .rs2_data_out(id_ex_rs2_data),
        .imm_out(id_ex_imm),
        .branch_target_out(id_ex_branch_target),
        .mem_read_out(id_ex_mem_read),
        .mem_write_out(id_ex_mem_write),
        .reg_write_out(id_ex_reg_write),
        .rs1_addr_out(id_ex_rs1_addr),
        .rs2_addr_out(id_ex_rs2_addr),
        .rd_addr_out(id_ex_rd_addr),
        .funct3_out(id_ex_funct3),
        .funct7_out(id_ex_funct7),
        .opcode_out(id_ex_opcode),
        .alu_src_out(id_ex_alu_src),
        .branch_out(id_ex_branch),
        .jump_out(id_ex_jump),
        .mem_to_reg_out(id_ex_mem_to_reg)
    );
    
    // Execute stage
    execute execute_stage(
        .clk(clk),
        .rst(rst),
        .pc_in(id_ex_pc),
        .rs1_data(id_ex_rs1_data),
        .rs2_data(id_ex_rs2_data),
        .imm(id_ex_imm),
        .branch_target(id_ex_branch_target), // 64 bits
        .mem_read(id_ex_mem_read),
        .mem_write(id_ex_mem_write),
        .reg_write(id_ex_reg_write),
        .rs1_addr(id_ex_rs1_addr),
        .rs2_addr(id_ex_rs2_addr),
        .rd_addr(id_ex_rd_addr),            // 5 bits
        .funct3(id_ex_funct3),              // 3 bits
        .funct7(id_ex_funct7),              // 7 bits
        .opcode(id_ex_opcode),
        .alu_src(id_ex_alu_src),
        .branch(id_ex_branch),
        .jump(id_ex_jump),
        .mem_to_reg(id_ex_mem_to_reg),
        .alu_result(ex_alu_result),
        .mem_address(ex_mem_address),
        .mem_write_data(ex_mem_write_data),
        .branch_taken(ex_branch_taken),
        .jump_target(ex_jump_target),
        .branch_target_out(ex_branch_target_out), // 64 bits
        .reg_write_out(ex_reg_write),
        .rd_addr_out(ex_rd_addr_out),            // 5 bits
        .mem_read_out(ex_mem_read),
        .mem_write_out(ex_mem_write),
        .funct3_out(ex_funct3_out),              // 3 bits
        .funct7_out(ex_funct7_out),              // 7 bits
        .mem_to_reg_out(ex_mem_to_reg)
    );

    // EX/MEM pipeline register
    ex_mem_register ex_mem_register(
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .flush(flush),
        .alu_result_in(ex_alu_result),
        .mem_address_in(ex_mem_address),
        .mem_write_data_in(ex_mem_write_data),
        .branch_taken_in(ex_branch_taken),
        .jump_target_in(ex_jump_target),
        .branch_target_in(ex_branch_target_out), // 64 bits
        .reg_write_in(ex_reg_write),
        .rd_addr_in(ex_rd_addr_out),            // 5 bits
        .funct3_in(ex_funct3_out),              // 3 bits
        .funct7_in(ex_funct7_out),              // 7 bits
        .mem_read_in(ex_mem_read),
        .mem_write_in(ex_mem_write),
        .mem_to_reg_in(ex_mem_to_reg),
        .alu_result_out(ex_mem_alu_result),
        .mem_address_out(ex_mem_mem_address),
        .mem_write_data_out(ex_mem_mem_write_data),
        .branch_taken_out(ex_mem_branch_taken),
        .jump_target_out(ex_mem_jump_target),
        .branch_target_out(ex_mem_branch_target), // 64 bits
        .reg_write_out(ex_mem_reg_write),
        .rd_addr_out(ex_mem_rd_addr),           // 5 bits
        .funct3_out(ex_mem_funct3),             // 3 bits
        .funct7_out(ex_mem_funct7),             // 7 bits
        .mem_read_out(ex_mem_mem_read),
        .mem_write_out(ex_mem_mem_write),
        .mem_to_reg_out(ex_mem_mem_to_reg)
    );
   
    // Memory stage
    memory memory_stage(
        .clk(clk),
        .rst(rst),
        .alu_result(ex_mem_alu_result),
        .mem_address(ex_mem_mem_address),
        .mem_write_data(ex_mem_mem_write_data),
        .branch_taken(ex_mem_branch_taken),
        .jump_target(ex_mem_jump_target),   
        .reg_write(ex_mem_reg_write),
        .rd_addr(ex_mem_rd_addr),
        .funct3(ex_mem_funct3),
        .mem_read(ex_mem_mem_read),          
        .mem_write(ex_mem_mem_write),        
        .mem_to_reg(ex_mem_mem_to_reg),      
        .mem_read_data(mem_read_data),           
        .mem_result(mem_result),              
        .reg_write_out(mem_reg_write),        
        .rd_addr_out(mem_rd_addr),
        .mem_to_reg_out(mem_mem_to_reg)      
    );
    
    // MEM/WB pipeline register
    mem_wb_register mem_wb_register(
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .flush(flush),
        .mem_result_in(mem_result),           
        .reg_write_in(mem_reg_write),         
        .rd_addr_in(mem_rd_addr),             
        .mem_to_reg_in(mem_mem_to_reg),      
        .mem_result_out(mem_wb_mem_result),
        .reg_write_out(mem_wb_reg_write),
        .rd_addr_out(mem_wb_rd_addr),
        .mem_to_reg_out(mem_wb_mem_to_reg)   
    );
    
    // Writeback stage
    writeback writeback_stage(
        .mem_result(mem_wb_mem_result),
        .reg_write(mem_wb_reg_write),
        .rd_addr(mem_wb_rd_addr),
        .mem_to_reg(mem_wb_mem_to_reg),      
        .write_back_data(write_back_data),
        .write_back_addr(write_back_addr),
        .reg_write_back(write_back_enable)
    );

endmodule