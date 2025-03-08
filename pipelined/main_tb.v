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
        // Program to test ALU, Branch, and Jump operations with hazard handling

        // 1. Basic ALU operations
        cpu.fetch_stage.imem.mem[0] = 32'h00000013;  // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[1] = 32'h00100093;  // addi x1, x0, 1    # x1 = 1
        cpu.fetch_stage.imem.mem[2] = 32'h00300113;  // addi x2, x0, 3    # x2 = 3
        cpu.fetch_stage.imem.mem[3] = 32'h00000013;  // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[4] = 32'h002081b3;  // add  x3, x1, x2   # x3 = x1 + x2 = 4
        cpu.fetch_stage.imem.mem[5] = 32'h00000013;  // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[6] = 32'h40208233;  // sub  x4, x1, x2   # x4 = x1 - x2 = -2
        cpu.fetch_stage.imem.mem[7] = 32'h00000013;  // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[8] = 32'h0020f2b3;  // and  x5, x1, x2   # x5 = x1 & x2 = 1
        cpu.fetch_stage.imem.mem[9] = 32'h00000013;  // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[10] = 32'h0020e333; // or   x6, x1, x2   # x6 = x1 | x2 = 3
        cpu.fetch_stage.imem.mem[11] = 32'h00000013; // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[12] = 32'h0020c3b3; // xor  x7, x1, x2   # x7 = x1 ^ x2 = 2
        cpu.fetch_stage.imem.mem[13] = 32'h00000013; // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[14] = 32'h00209433; // sll  x8, x1, x2   # x8 = x1 << x2 = 8
        cpu.fetch_stage.imem.mem[15] = 32'h00000013; // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[16] = 32'h002081b3;  // add  x3, x1, x2   # x3 = x1 + x2 = 4

        // 2. Immediate operations
        cpu.fetch_stage.imem.mem[17] = 32'h00a00513;  // addi x10, x0, 10   # x10 = 10
        cpu.fetch_stage.imem.mem[18] = 32'h00000013;  // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[19] = 32'hfff00593; // addi x11, x0, -1   # x11 = -1

        // 3. Branch operations - test all branch types
        cpu.fetch_stage.imem.mem[20] = 32'h00c00613; // addi x12, x0, 12   # x12 = 12
        cpu.fetch_stage.imem.mem[21] = 32'h00000013; // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[22] = 32'h00c00693; // addi x13, x0, 12   # x13 = 12
        cpu.fetch_stage.imem.mem[23] = 32'h00000013; // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[24] = 32'h00d60663; // beq  x12, x13, 12  # Branch taken to PC+12
        cpu.fetch_stage.imem.mem[25] = 32'h00000013; // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[26] = 32'h00000013; // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[27] = 32'h00500713; // addi x14, x0, 5    # x14 = 5 (after branch)

        // 4. Jump operations - test JAL and JALR
        cpu.fetch_stage.imem.mem[28] = 32'h008000ef; // jal  x1, 8        # Jump to PC+8, x1 = PC+4
        cpu.fetch_stage.imem.mem[29] = 32'h00000013; // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[30] = 32'h00000793; // addi x15, x0, 0   # x15 = 0 (after jump)
        cpu.fetch_stage.imem.mem[31] = 32'h00000013; // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[32] = 32'h00000813; // addi x16, x0, 0   # x16 = 0
        cpu.fetch_stage.imem.mem[33] = 32'h00000013; // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[34] = 32'h01000893; // addi x17, x0, 16  # x17 = 16
        cpu.fetch_stage.imem.mem[35] = 32'h00000013; // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[36] = 32'h000880e7; // jalr x1, x17, 0   # Jump to x17, x1 = PC+4

        // 5. Load/Store operations
        cpu.fetch_stage.imem.mem[37] = 32'h00100913; // addi x18, x0, 1   # x18 = 1
        cpu.fetch_stage.imem.mem[38] = 32'h00000013; // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[39] = 32'h01252023; // sw   x18, 0(x10)  # Store x18 to mem[x10]
        cpu.fetch_stage.imem.mem[40] = 32'h00000013; // NOP (avoid hazard)
        cpu.fetch_stage.imem.mem[41] = 32'h00052983; // lw   x19, 0(x10)  # Load from mem[x10] to x19

        // End with an infinite loop
        cpu.fetch_stage.imem.mem[42] = 32'h0000006f; // jal  x0, 0        # Infinite loop
    end
    
    // Termination
    initial begin
        #500;
        $display("Test completed");
        $finish;
    end
    
    // Monitoring key signals
    initial begin
        $display("Time | PC  | Instruction | ALUResult | Branch | Jump | Registers");
        $display("-----------------------------------------------------------------");
        $monitor("%4d | %3d | %h | %d | %b | %d |\n  x1=%d x2=%d x3=%d \n x4=%d x5=%d x6=%d x7=%d",
            $time, // Time in decimal
            cpu.fetch_stage.pc, // PC in decimal
            cpu.if_id_register.instruction_out, // Instruction in hex
            (cpu.execute_stage.alu_result), // ALU result in decimal
            cpu.execute_stage.branch_taken, // Branch signal in binary
            (cpu.execute_stage.jump_target), // Jump target in decimal
            (cpu.decode_stage.reg_file.registers[1]), // Register x1 in decimal
            (cpu.decode_stage.reg_file.registers[2]), // Register x2 in decimal
            (cpu.decode_stage.reg_file.registers[3]), // Register x3 in decimal
            (cpu.decode_stage.reg_file.registers[4]), // Register x4 in decimal
            (cpu.decode_stage.reg_file.registers[5]), // Register x5 in decimal
            (cpu.decode_stage.reg_file.registers[6]), // Register x6 in decimal
            (cpu.decode_stage.reg_file.registers[7])  // Register x7 in signed decimal
        );
    end
    
    // Additional monitoring for key events
    always @(posedge clk) begin
        // Monitor branch instructions
        if (cpu.execute_stage.branch_taken) begin
            $display("\n BRANCH TAKEN at time %4d: PC=%h, Target=%h", 
                $time, 
                cpu.fetch_stage.pc, 
                cpu.execute_stage.jump_target
            );
        end
        
        // Monitor jump instructions
        if (cpu.decode_stage.jump && !cpu.execute_stage.branch) begin
            $display("\n JUMP DETECTED at time %4d: PC=%h, Target=%h", 
                $time, 
                cpu.fetch_stage.pc, 
                cpu.execute_stage.jump_target
            );
        end
        
        // Monitor ALU operations (R-type instructions)
        if (cpu.if_id_register.instruction_out[6:0] == 7'b0110011) begin
            $display("\n ALU R-TYPE at time %4d: funct3=%b, funct7=%b, Result=%h", 
                $time, 
                cpu.decode_stage.funct3,
                cpu.decode_stage.funct7,
                cpu.execute_stage.alu_result
            );
        end
        
        // Monitor ALU immediate operations (I-type instructions)
        if (cpu.if_id_register.instruction_out[6:0] == 7'b0010011) begin
            $display("\n ALU I-TYPE at time %4d: funct3=%b, imm=%h, Result=%h", 
                $time, 
                cpu.decode_stage.funct3,
                cpu.decode_stage.imm[11:0],
                cpu.execute_stage.alu_result
            );
        end
    end
endmodule