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
        $dumpfile("branch_jump_test.vcd");
        $dumpvars(0, RISC_V_CPU_Test);
    end
    
    // Clock Generation (2ns period)
    initial begin
        clk = 0;
        forever #1 clk = ~clk;
    end
    
    integer i;
    initial begin
        // Apply reset
        reset = 1;
        #6 reset = 0;
        
        // Initialize memory with all zeros first
        for (i = 0; i < 1024; i = i + 1) begin
            cpu.fetch_stage.imem.mem[i] = 32'h00000000;
        end
        
        // Load test program
        // 1. NOP (No Operation)
        cpu.fetch_stage.imem.mem[0] = 32'b0000000_00000_00000_000_00000_0010011; // nop (addi x0, x0, 0)

        // 2. addi x1, x0, 1 (I-type)
        cpu.fetch_stage.imem.mem[1] = 32'b0000000_00001_00000_000_00001_0010011; // addi x1, x0, 1

        // 3. addi x2, x0, 2 (I-type)
        cpu.fetch_stage.imem.mem[2] = 32'b0000000_00010_00000_000_00010_0010011; // addi x2, x0, 2

        // 4. addi x3, x0, 3 (I-type)
        cpu.fetch_stage.imem.mem[3] = 32'b0000000_00011_00000_000_00011_0010011; // addi x3, x0, 3

        // 5. beq x1, x2, 8 (Branch if x1 == x2, PC + 8)
        cpu.fetch_stage.imem.mem[4] = 32'b0000000_00010_00001_000_01000_1100011; // beq x1, x2, 8

        // 6. bne x1, x2, 8 (Branch if x1 != x2, PC + 8)
        cpu.fetch_stage.imem.mem[5] = 32'b0000000_00010_00001_001_01000_1100011; // bne x1, x2, 8

        // 7. blt x1, x2, 8 (Branch if x1 < x2, PC + 8)
        cpu.fetch_stage.imem.mem[6] = 32'b0000000_00010_00001_100_01000_1100011; // blt x1, x2, 8

        // 8. bge x1, x2, 8 (Branch if x1 >= x2, PC + 8)
        cpu.fetch_stage.imem.mem[7] = 32'b0000000_00010_00001_101_01000_1100011; // bge x1, x2, 8

        // 9. jal x1, 16 (Jump to PC + 16, save return address in x1)
        cpu.fetch_stage.imem.mem[8] = 32'b0000000_10000_00000_000_00001_1101111; // jal x1, 16

        // 10. jalr x2, x1, 0 (Jump to address in x1 + 0, save return address in x2)
        cpu.fetch_stage.imem.mem[9] = 32'b0000000_00000_00001_000_00010_1100111; // jalr x2, x1, 0

        // 11. Infinite loop
        cpu.fetch_stage.imem.mem[10] = 32'b0000000_00000_00000_000_00000_1101111; // jal x0, 0 (infinite loop)
    end
    
    // Termination
    initial begin
        #100;
        $display("\n===== Test completed =====");
        $finish;
    end
    
    // Monitoring
    initial begin
        $display("\n+-------+--------+---------------+----------+--------+----------+");
        $display("| Time  |   PC   |  Instruction  | ALUResult| Branch |   Jump   |");
        $display("+-------+--------+---------------+----------+--------+----------+");
        $monitor("| %4t ns | PC  %0d | INST  %b | RES %08h | B ?  %b    | JUMP %08h |\n| Register Values: x1=0x%08h x2=0x%08h x3=0x%08h |\n",
            $time, // Time in decimal with ns unit
            cpu.fetch_stage.pc, // PC in decimal
            cpu.if_id_register.instruction_out, // Instruction in binary
            cpu.execute_stage.alu_result, // ALU result in hex
            cpu.execute_stage.branch_taken, // Branch signal in binary
            cpu.execute_stage.jump_target, // Jump target in hex
            cpu.decode_stage.reg_file.registers[1], // Register x1 in hex
            cpu.decode_stage.reg_file.registers[2], // Register x2 in hex
            cpu.decode_stage.reg_file.registers[3]  // Register x3 in hex
        );
    end
endmodule