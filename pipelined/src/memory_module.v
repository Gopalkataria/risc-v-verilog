module memory(
    input clk,                          
    input rst,                          
    input [63:0] alu_result,            // ALU result from EX/MEM stage
    input [63:0] mem_address,           // Memory address from EX/MEM stage
    input [63:0] mem_write_data,        // Data to write to memory (for sw)
    input branch_taken,                 // Branch taken signal from EX/MEM stage
    input [63:0] jump_target,           // Jump target address from EX/MEM stage
    input reg_write,                    // Register write enable from EX/MEM stage
    input [4:0] rd_addr,                // Destination register address from EX/MEM stage
    input [2:0] funct3,                 // funct3 field (for memory access type)
    input [6:0] funct7,                 // funct7 field (for ALU operations)
    input mem_read,                     // Memory read signal (for lw) from EX/MEM stage
    input mem_write,                    // Memory write signal (for sw) from EX/MEM stage
    input mem_to_reg,                   // Memory-to-register signal (for lw) from EX/MEM stage
    output wire [63:0] mem_read_data,   // Data read from memory (for lw)
    output reg [63:0] mem_result,       // Result to be written to register file
    output reg reg_write_out,           // Register write enable to MEM/WB stage
    output reg [4:0] rd_addr_out,       // Destination register address to MEM/WB stage
    output reg mem_to_reg_out           // Memory-to-register signal to MEM/WB stage
);
     // Data memory instantiation
    data_memory dmem(
        .clk(clk),
        .address(mem_address),
        .write_data(mem_write_data),
        .mem_read(mem_read),            // Memory read signal (for lw)
        .mem_write(mem_write),          // Memory write signal (for sw)
        .read_data(mem_read_data)       // Data read from memory
    );

    // Memory result selection
    always @(*) begin
        if (mem_to_reg) begin
            // For lw, the result comes from memory
            case (funct3)
                3'b000: mem_result = {{56{mem_read_data[7]}}, mem_read_data[7:0]}; // lb (load byte, sign-extended)
                3'b001: mem_result = {{48{mem_read_data[15]}}, mem_read_data[15:0]}; // lh (load halfword, sign-extended)
                3'b010: mem_result = {{32{mem_read_data[31]}}, mem_read_data[31:0]}; // lw (load word, sign-extended)
                3'b011: mem_result = mem_read_data; // ld (load doubleword)
                3'b100: mem_result = {56'b0, mem_read_data[7:0]}; // lbu (load byte, zero-extended)
                3'b101: mem_result = {48'b0, mem_read_data[15:0]}; // lhu (load halfword, zero-extended)
                3'b110: mem_result = {32'b0, mem_read_data[31:0]}; // lwu (load word, zero-extended)
                default: mem_result = 64'b0; // Default (invalid)
            endcase
        end else begin
            // For other instructions, the result comes from the ALU
            mem_result = alu_result;
        end
    end

    // Pipeline signals
    always @(*) begin
        reg_write_out = reg_write;      // Pass register write enable to MEM/WB stage
        rd_addr_out = rd_addr;         // Pass destination register address to MEM/WB stage
        mem_to_reg_out = mem_to_reg;   // Pass memory-to-register signal to MEM/WB stage
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

