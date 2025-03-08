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
        $dumpfile("memory_test.vcd");
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
        
        // Initialize instruction memory with all zeros first
        for (i = 0; i < 1024; i = i + 1) begin
            cpu.fetch_stage.imem.mem[i] = 32'h00000000;
        end
        
        // Hardcode some values in data memory for testing
        cpu.memory_stage.dmem.mem[0]  = 64'h1234567812345678; // Word at address 0
        cpu.memory_stage.dmem.mem[1]  = 64'h9ABCDEF09ABCDEF0; // Word at address 4
        cpu.memory_stage.dmem.mem[2]  = 64'h1122334411223344; // Word at address 8
        cpu.memory_stage.dmem.mem[3]  = 64'haaaaaaaaaaaaaaaa; // Word at address 12
        cpu.memory_stage.dmem.mem[4]  = 64'hbbbbbbbbbbbbbbbb; // Word at address 16
        cpu.memory_stage.dmem.mem[5]  = 64'hcccccccccccccccc; // Word at address 20

        // Load test program into instruction memory
        // 1. NOP (No Operation)
        cpu.fetch_stage.imem.mem[0] = 32'b0000000_00000_00000_000_00000_0010011; // nop (addi x0, x0, 0)

         cpu.fetch_stage.imem.mem[1] = 32'b0000000_00000_00000_010_00001_0000011; // lw x1, 0(x0)

        cpu.fetch_stage.imem.mem[2] = 32'b0000000_00100_00000_001_00010_0000011; // lh x2, 4(x0)

        cpu.fetch_stage.imem.mem[3] = 32'b0000000_01000_00000_000_00011_0000011; // lb x3, 8(x0)

        cpu.fetch_stage.imem.mem[4] = 32'b0000000_00000_00000_110_00100_0000011; // lwu x4, 0(x0)

        cpu.fetch_stage.imem.mem[5] = 32'b0000000_00100_00000_101_00101_0000011; // lhu x5, 4(x0)

        cpu.fetch_stage.imem.mem[6] = 32'b0000000_01000_00000_100_00110_0000011; // lbu x6, 8(x0)

        cpu.fetch_stage.imem.mem[7] = 32'b0000000_00001_00000_010_01100_0100011; // sw x1, 12(x0)

        cpu.fetch_stage.imem.mem[8] = 32'b0000000_00010_00000_001_10000_0100011; // sh x2, 16(x0)

        cpu.fetch_stage.imem.mem[9] = 32'b0000000_00011_00000_000_10100_0100011; // sb x3, 20(x0)

        // 8. Infinite loop
        // cpu.fetch_stage.imem.mem[9] = 32'b0000000_00000_00000_000_00000_1101111; // jal x0, 0 (infinite loop)
    end
    
    // Termination
    initial begin
        #70;
        $display("\n===== Test completed =====");
        $finish;
    end
    
    // Monitoring
    initial begin
        $display("\n+-------+--------+---------------+----------+--------+----------+");
        $display("| Time  |   PC   |  Instruction  | ALUResult| Branch |   Jump   |");
        $display("+-------+--------+---------------+----------+--------+----------+");
        $monitor("| %4t ns | PC  %0d | INST  %b | RES %08h | B ?  %b    | JUMP %08h |\n| Register Values: x1=0x%08h x2=0x%08h x3=0x%08h |\n| Register Values: x4=0x%08h x5=0x%08h x6=0x%08h |\n | Memory Values: [0]=0x%08h [4]=0x%08h [8]=0x%08h [12]=0x%08h [16]=0x%08h [20]=0x%08h |\n",
            $time, // Time in decimal with ns unit
            cpu.fetch_stage.pc, // PC in decimal
            cpu.if_id_register.instruction_out, // Instruction in binary
            cpu.execute_stage.alu_result, // ALU result in hex
            cpu.execute_stage.branch_taken, // Branch signal in binary
            cpu.execute_stage.jump_target, // Jump target in hex
            cpu.decode_stage.reg_file.registers[1], // Register x1 in hex
            cpu.decode_stage.reg_file.registers[2], // Register x2 in hex
            cpu.decode_stage.reg_file.registers[3], // Register x3 in hex
            cpu.decode_stage.reg_file.registers[4], // Register x1 in hex
            cpu.decode_stage.reg_file.registers[5], // Register x2 in hex
            cpu.decode_stage.reg_file.registers[6], // Register x3 in hex
            cpu.memory_stage.dmem.mem[0], // Memory at address 0
            cpu.memory_stage.dmem.mem[1], // Memory at address 4
            cpu.memory_stage.dmem.mem[2], // Memory at address 8
            cpu.memory_stage.dmem.mem[3], // Memory at address 12
            cpu.memory_stage.dmem.mem[4], // Memory at address 16
            cpu.memory_stage.dmem.mem[5]  // Memory at address 20
        );
    end
endmodule