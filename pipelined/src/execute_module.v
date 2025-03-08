module execute(
    input clk,
    input rst,
    input [63:0] pc_in,
    input [63:0] rs1_data,
    input [63:0] rs2_data,
    input [63:0] imm,
    input [63:0] branch_target,
    input mem_read,
    input mem_write,
    input reg_write,
    input [4:0] rs1_addr,
    input [4:0] rs2_addr,
    input [4:0] rd_addr,
    input [2:0] funct3,
    input [6:0] funct7,
    input [6:0] opcode,
    input alu_src,
    input branch,
    input jump,
    input mem_to_reg,
    output wire [63:0] alu_result,
    output wire [63:0] mem_address,
    output wire [63:0] mem_write_data,
    output wire branch_taken,
    output wire [63:0] jump_target,
    output wire reg_write_out,
    output wire [4:0] rd_addr_out,
    output wire mem_read_out,          // Pass mem_read to next stage
    output wire mem_write_out,         // Pass mem_write to next stage
    output wire [2:0] funct3_out,      // Pass funct3 to next stage
    output wire [6:0] funct7_out,      // Pass funct7 to next stage
    output wire mem_to_reg_out         // Pass mem_to_reg to next stage
);
    // ALU operand selection
    wire [63:0] alu_operand2 = alu_src ? imm : rs2_data;

    // ALU instantiation
    alu_64bit alu(
        .funct3(funct3),
        .funct7(funct7),
        .a(rs1_data),
        .b(alu_operand2),
        .result(alu_result)
    );

    // Branch logic
    reg branch_taken_reg;
    always @(*) begin
        branch_taken_reg = 1'b0;
        if (branch) begin
            case (funct3)
                3'b000: branch_taken_reg = (rs1_data == rs2_data);    // beq
                3'b001: branch_taken_reg = (rs1_data != rs2_data);    // bne
                3'b100: branch_taken_reg = ($signed(rs1_data) < $signed(rs2_data));  // blt
                3'b101: branch_taken_reg = ($signed(rs1_data) >= $signed(rs2_data)); // bge
                3'b110: branch_taken_reg = (rs1_data < rs2_data);     // bltu
                3'b111: branch_taken_reg = (rs1_data >= rs2_data);    // bgeu
                default: branch_taken_reg = 1'b0;
            endcase
        end
    end

    // Jump logic
    reg [63:0] jump_target_reg;
    reg jump_taken;
    always @(*) begin
        jump_target_reg = 64'b0;
        jump_taken = 1'b0;
        
        if (jump) begin
            if (opcode == 7'b1101111) begin  // jal
                jump_target_reg = pc_in + imm;
                jump_taken = 1'b1;
            end else if (opcode == 7'b1100111 && funct3 == 3'b000) begin  // jalr
                jump_target_reg = (rs1_data + imm) & ~64'h1; // Clear LSB for alignment
                jump_taken = 1'b1;
            end
        end
    end

    // Memory address calculation
    assign mem_address = rs1_data + imm;

    // Memory write data formatting
    reg [63:0] mem_write_data_reg;
    always @(*) begin
        case (funct3)
            3'b000: mem_write_data_reg = {56'b0, rs2_data[7:0]};   // sb (store byte)
            3'b001: mem_write_data_reg = {48'b0, rs2_data[15:0]};  // sh (store halfword)
            3'b010: mem_write_data_reg = {32'b0, rs2_data[31:0]};  // sw (store word)
            3'b011: mem_write_data_reg = rs2_data;                 // sd (store doubleword)
            default: mem_write_data_reg = rs2_data;
        endcase
    end
    assign mem_write_data = mem_write_data_reg;

    // Pipeline signals
    assign branch_taken = branch_taken_reg | jump_taken;
    assign jump_target = jump_target_reg;
    assign reg_write_out = reg_write;
    assign rd_addr_out = rd_addr;

    // Pass memory-related signals to next stage
    assign mem_read_out = mem_read;
    assign mem_write_out = mem_write;
    assign funct3_out = funct3;
    assign funct7_out = funct7;
    assign mem_to_reg_out = mem_to_reg;
endmodule


module ex_mem_register(
    input clk,                          
    input rst,                          
    input stall,                        
    input flush,                        
    input [63:0] alu_result_in,         // ALU result from execute stage
    input [63:0] mem_address_in,        // Memory address from execute stage
    input [63:0] mem_write_data_in,     // Data to write to memory (for sw)
    input branch_taken_in,              // Branch taken signal
    input [63:0] jump_target_in,        // Jump target address
    input reg_write_in,                 // Register write enable
    input [4:0] rd_addr_in,             // Destination register address
    input [2:0] funct3_in,              // funct3 field (for memory access type)
    input [6:0] funct7_in,              // funct7 field (for ALU operations)
    input mem_read_in,                  // Memory read signal (for lw)
    input mem_write_in,                 // Memory write signal (for sw)
    input mem_to_reg_in,                // Memory-to-register signal (for lw)
    output reg [63:0] alu_result_out,   // ALU result to memory stage
    output reg [63:0] mem_address_out,  // Memory address to memory stage
    output reg [63:0] mem_write_data_out, // Data to write to memory (for sw)
    output reg branch_taken_out,        // Branch taken signal to memory stage
    output reg [63:0] jump_target_out,  // Jump target address to memory stage
    output reg reg_write_out,           // Register write enable to memory stage
    output reg [4:0] rd_addr_out,       // Destination register address to memory stage
    output reg [2:0] funct3_out,        // funct3 field to memory stage
    output reg [6:0] funct7_out,        // funct7 field to memory stage
    output reg mem_read_out,            // Memory read signal to memory stage
    output reg mem_write_out,           // Memory write signal to memory stage
    output reg mem_to_reg_out           // Memory-to-register signal to memory stage
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset all outputs
            alu_result_out <= 64'b0;
            mem_address_out <= 64'b0;
            mem_write_data_out <= 64'b0;
            branch_taken_out <= 1'b0;
            jump_target_out <= 64'b0;
            reg_write_out <= 1'b0;
            rd_addr_out <= 5'b0;
            funct3_out <= 3'b0;
            funct7_out <= 7'b0;
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
        end else if (flush) begin
            // Flush all outputs (e.g., on branch misprediction)
            alu_result_out <= 64'b0;
            mem_address_out <= 64'b0;
            mem_write_data_out <= 64'b0;
            branch_taken_out <= 1'b0;
            jump_target_out <= 64'b0;
            reg_write_out <= 1'b0;
            rd_addr_out <= 5'b0;
            funct3_out <= 3'b0;
            funct7_out <= 7'b0;
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
        end else if (stall) begin
            // Stall: retain current values
            alu_result_out <= alu_result_out;
            mem_address_out <= mem_address_out;
            mem_write_data_out <= mem_write_data_out;
            branch_taken_out <= branch_taken_out;
            jump_target_out <= jump_target_out;
            reg_write_out <= reg_write_out;
            rd_addr_out <= rd_addr_out;
            funct3_out <= funct3_out;
            funct7_out <= funct7_out;
            mem_read_out <= mem_read_out;
            mem_write_out <= mem_write_out;
            mem_to_reg_out <= mem_to_reg_out;
        end else begin
            // Normal operation: pass inputs to outputs
            alu_result_out <= alu_result_in;
            mem_address_out <= mem_address_in;
            mem_write_data_out <= mem_write_data_in;
            branch_taken_out <= branch_taken_in;
            jump_target_out <= jump_target_in;
            reg_write_out <= reg_write_in;
            rd_addr_out <= rd_addr_in;
            funct3_out <= funct3_in;
            funct7_out <= funct7_in;
            mem_read_out <= mem_read_in;
            mem_write_out <= mem_write_in;
            mem_to_reg_out <= mem_to_reg_in;
        end
    end
endmodule