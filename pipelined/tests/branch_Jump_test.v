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
        $dumpfile("stall_flush_test.vcd");
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
               

    // 1-3. NOPs (No Operation)
cpu.fetch_stage.imem.mem[0] = 32'b0000000_00000_00000_000_00000_0010011; // nop (addi x0, x0, 0)
cpu.fetch_stage.imem.mem[1] = 32'b0000000_00000_00000_000_00000_0010011; // nop (addi x0, x0, 0)
cpu.fetch_stage.imem.mem[2] = 32'b0000000_00000_00000_000_00000_0010011; // nop (addi x0, x0, 0)
//  x1 = 10 
cpu.fetch_stage.imem.mem[3] = 32'b0000000_01010_00000_000_00001_0010011; // addi x1, x0, 10
//  x2 = 20
cpu.fetch_stage.imem.mem[4] = 32'b0000000_10100_00000_000_00010_0010011; // addi x2, x0, 20
//  x3 = 20
cpu.fetch_stage.imem.mem[5] = 32'b0000000_10100_00000_000_00011_0010011; // addi x3, x0, 20
// beq x1, x2, skip 4 instructions
// nop 
cpu.fetch_stage.imem.mem[6] = 32'b0000000_00000_00000_000_00000_0010011; // nop (addi x0, x0, 0)
// nop
cpu.fetch_stage.imem.mem[7] = 32'b0000000_00000_00000_000_00000_0010011; // nop (addi x0, x0, 0)
// nop
cpu.fetch_stage.imem.mem[8] = 32'b0000000_00000_00000_000_00000_0010011; // nop (addi x0, x0, 0)

cpu.fetch_stage.imem.mem[9] = 32'b0000000_00010_00001_000_00100_1100011; // beq x1, x2, 4
// beq x2, x3, skip 2 instructions
cpu.fetch_stage.imem.mem[10] = 32'b0000000_00011_00010_000_01000_1100011; // beq x2, x3, 8
//  x4 = 30
cpu.fetch_stage.imem.mem[11] = 32'b0000000_11110_00000_000_00100_0010011; // addi x4, x0, 30
//  x5 = 40
cpu.fetch_stage.imem.mem[12] = 32'b0000000_10100_00000_000_00101_0010011; // addi x5, x0, 40
//  x6 = 50
cpu.fetch_stage.imem.mem[13] = 32'b0000000_10010_00000_000_00110_0010011; // addi x6, x0, 50
//  x7 = 60
cpu.fetch_stage.imem.mem[14] = 32'b0000000_11100_00000_000_00111_0010011; // addi x7, x0, 60
//  x8 = 70
cpu.fetch_stage.imem.mem[15] = 32'b0000000_00110_00000_000_01000_0010011; // addi x8, x0, 70
// loop inf, beq x0, x0, 0 (infinite loop)
cpu.fetch_stage.imem.mem[16] = 32'b0000000_00000_00000_000_00000_1100011; // beq x0, x0, 0

        
    end
    
    // Termination
    initial begin
        #100;
        $display("\n===== Test completed =====");
        $finish;
    end
    
    // Monitoring
    initial begin
        $display("| Time  |   PC   |  Instruction  | ALUResult| Branch |   Jump   |");
        $monitor("| %4t ns | PC  %0d | INST  %b | RES %08h | B ?  %b    | JUMP %08h | flush %b , stall %0b |\n| Register Values: x1=0x%08h x2=0x%08h x3=0x%08h x4=0x%08h |\n| x5=0x%08h x6=0x%08h x7=0x%08h x8=0x%08h x9=0x%08h |\n| x10=0x%08h x11=0x%08h x12=0x%08h x13=0x%08h |\n",
            $time, // Time in decimal with ns unit
            cpu.fetch_stage.pc, // PC in decimal
            cpu.if_id_register.instruction_out, // Instruction in binary
            cpu.execute_stage.alu_result, // ALU result in hex
            cpu.execute_stage.branch_taken, // Branch signal in binary
            cpu.execute_stage.jump_target, // Jump target in hex
            cpu.flush, // Flush signal in binary
            cpu.stall, // Stall signal in binary
            cpu.decode_stage.reg_file.registers[1], /// x1 
            cpu.decode_stage.reg_file.registers[2], /// x2 
            cpu.decode_stage.reg_file.registers[3], /// x3 
            cpu.decode_stage.reg_file.registers[4], /// x4 
            cpu.decode_stage.reg_file.registers[5], /// x5 
            cpu.decode_stage.reg_file.registers[6], /// x6 
            cpu.decode_stage.reg_file.registers[7], /// x7 
            cpu.decode_stage.reg_file.registers[8], /// x8 
            cpu.decode_stage.reg_file.registers[9], /// x9 
            cpu.decode_stage.reg_file.registers[10], //  x10 
            cpu.decode_stage.reg_file.registers[11], //  x11 
            cpu.decode_stage.reg_file.registers[12], //  x12 
            cpu.decode_stage.reg_file.registers[13]  //  x13 
        );
    end
endmodule