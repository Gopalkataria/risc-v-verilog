`timescale 1ns / 1ps

module RISC_V_CPU_Test;
    reg clk;
    reg reset;
    
    // Instantiate the RISC-V CPU
    riscv_processor cpu (
        .clk(clk),
        .rst(reset)
    );
    
    // GTKWave Dump
    initial begin
        $dumpfile("risc_v_test.vcd");
        $dumpvars(0, RISC_V_CPU_Test);
    end
    
    // Clock Generation (2ns period)
    initial begin
        clk = 0;
        forever #1 clk = ~clk;
    end
    
    // Reset and Test Program Initialization
    integer i;
    initial begin
        // Apply reset
        reset = 1;
        #10 reset = 0;
        
        // Initialize memory with all zeros first
        for (i = 0; i < 1024; i = i + 1) begin
            cpu.fetch_stage.imem.mem[i] = 32'h00000000;
        end
        
        // Load test program
        // Program to test ALU, Branch, and Jump operations
        
        // 1. Basic ALU operations
       cpu.fetch_stage.imem.mem[0] = 32'b00000000101000000000000010010011;   // li x1, 10   -> ADDI x1, x0, 10
cpu.fetch_stage.imem.mem[1] = 32'b00000001010000000000000100010011;   // li x2, 20   -> ADDI x2, x0, 20
cpu.fetch_stage.imem.mem[2] = 32'b00000001111000000000000110010011;   // li x3, 30   -> ADDI x3, x0, 30
cpu.fetch_stage.imem.mem[3] = 32'b00000010101000000000001000010011;   // addi x4, x0, 42  -> ADDI x4, x0, 42
cpu.fetch_stage.imem.mem[4] = 32'b00000000001000010000001010110011;   // add x5, x1, x2  -> ADD x5, x1, x2
cpu.fetch_stage.imem.mem[5] = 32'b00000000001000001100001100110011;   // xor x6, x1, x2  -> XOR x6, x1, x2 (Independent instruction)
cpu.fetch_stage.imem.mem[6] = 32'b01000000001100101000001110110011;   // sub x7, x5, x3  -> SUB x7, x5, x3
cpu.fetch_stage.imem.mem[7] = 32'b00000000010100101000001010100011;   // sw x5, 0(x8)   -> SW x5, 0(x8)

// 2. Immediate operations
cpu.fetch_stage.imem.mem[8] = 32'b00000000101000000000001001010011;   // li x9, 10  -> ADDI x9, x0, 10
cpu.fetch_stage.imem.mem[9] = 32'b00000000000001000000001010000011;  // lw x10, 0(x8)  -> LW x10, 0(x8)

        
        // 3. Branch operations - test all branch types
        cpu.fetch_stage.imem.mem[11] = 32'h00c00613; // addi x12, x0, 12   # x12 = 12
        cpu.fetch_stage.imem.mem[12] = 32'h00c00693; // addi x13, x0, 12   # x13 = 12
        cpu.fetch_stage.imem.mem[13] = 32'h00d60663; // beq  x12, x13, 12  # Branch taken to PC+12
        cpu.fetch_stage.imem.mem[14] = 32'h00000013; // addi x0, x0, 0     # NOP (should be skipped)
        cpu.fetch_stage.imem.mem[15] = 32'h00000013; // addi x0, x0, 0     # NOP (should be skipped)
        cpu.fetch_stage.imem.mem[16] = 32'h00500713; // addi x14, x0, 5    # x14 = 5 (after branch)
        
        // 4. Jump operations - test JAL and JALR
        cpu.fetch_stage.imem.mem[17] = 32'h008000ef; // jal  x1, 8        # Jump to PC+8, x1 = PC+4
        cpu.fetch_stage.imem.mem[18] = 32'h00000013; // addi x0, x0, 0    # NOP (should be skipped)
        cpu.fetch_stage.imem.mem[19] = 32'h00000793; // addi x15, x0, 0   # x15 = 0 (after jump)
        cpu.fetch_stage.imem.mem[20] = 32'h00000813; // addi x16, x0, 0   # x16 = 0
        cpu.fetch_stage.imem.mem[21] = 32'h01000893; // addi x17, x0, 16  # x17 = 16
        cpu.fetch_stage.imem.mem[22] = 32'h000880e7; // jalr x1, x17, 0   # Jump to x17, x1 = PC+4
        
        // 5. Load/Store operations
        cpu.fetch_stage.imem.mem[23] = 32'h00100913; // addi x18, x0, 1   # x18 = 1
        cpu.fetch_stage.imem.mem[24] = 32'h01252023; // sw   x18, 0(x10)  # Store x18 to mem[x10]
        cpu.fetch_stage.imem.mem[25] = 32'h00052983; // lw   x19, 0(x10)  # Load from mem[x10] to x19
        
        // End with an infinite loop
        cpu.fetch_stage.imem.mem[26] = 32'h0000006f; // jal  x0, 0        # Infinite loop
    end
    
    // Termination
    initial begin
        #500;
        $display("Test completed");
        $finish;
    end
    
    // Monitoring key signals
    initial begin
        $display("Time | PC | Instruction | ALUResult | Branch | Jump | Registers");
        $display("----------------------------------------------------------------");
        $monitor("%4d | %h | %h | %h | %b | %h | x1=%h x2=%h x3=%h x4=%h x5=%h x6=%h x7=%h",
            $time,
            cpu.fetch_stage.pc,
            cpu.if_id_register.instruction_out,
            cpu.execute_stage.alu_result,
            cpu.execute_stage.branch_taken,
            cpu.execute_stage.jump_target,
            cpu.decode_stage.reg_file.registers[1],
            cpu.decode_stage.reg_file.registers[2],
            cpu.decode_stage.reg_file.registers[3],
            cpu.decode_stage.reg_file.registers[4],
            cpu.decode_stage.reg_file.registers[5],
            cpu.decode_stage.reg_file.registers[6],
            cpu.decode_stage.reg_file.registers[7]
        );
    end
    
    // Additional monitoring for key events
    always @(posedge clk) begin
        // Monitor branch instructions
        if (cpu.execute_stage.branch_taken) begin
            $display("BRANCH TAKEN at time %4d: PC=%h, Target=%h", 
                $time, 
                cpu.fetch_stage.pc, 
                cpu.execute_stage.jump_target
            );
        end
        
        // Monitor jump instructions
        if (cpu.decode_stage.jump && !cpu.execute_stage.branch) begin
            $display("JUMP DETECTED at time %4d: PC=%h, Target=%h", 
                $time, 
                cpu.fetch_stage.pc, 
                cpu.execute_stage.jump_target
            );
        end
        
        // Monitor ALU operations (R-type instructions)
        if (cpu.if_id_register.instruction_out[6:0] == 7'b0110011) begin
            $display("ALU R-TYPE at time %4d: funct3=%b, funct7=%b, Result=%h", 
                $time, 
                cpu.decode_stage.funct3,
                cpu.decode_stage.funct7,
                cpu.execute_stage.alu_result
            );
        end
        
        // Monitor ALU immediate operations (I-type instructions)
        if (cpu.if_id_register.instruction_out[6:0] == 7'b0010011) begin
            $display("ALU I-TYPE at time %4d: funct3=%b, imm=%h, Result=%h", 
                $time, 
                cpu.decode_stage.funct3,
                cpu.decode_stage.imm[11:0],
                cpu.execute_stage.alu_result
            );
        end
    end
endmodule