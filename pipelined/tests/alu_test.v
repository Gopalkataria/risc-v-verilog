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
        $dumpfile("alu_test.vcd");
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

        // 4. addi x3, x3, 3 (I-type)
        cpu.fetch_stage.imem.mem[3] = 32'b0000000_00011_00011_000_00011_0010011; // addi x3, x3, 3

        // 5. nop
        cpu.fetch_stage.imem.mem[4] = 32'b0000000_00000_00000_000_00000_0010011; // nop (addi x0, x0, 0)

        // 6. nop 
        cpu.fetch_stage.imem.mem[5] = 32'b0000000_00000_00000_000_00000_0010011; // nop (addi x0, x0, 0)

        // 6. sub x5, x1, x2 (R-type)
        cpu.fetch_stage.imem.mem[6] = 32'b0100000_00010_00001_000_00101_0110011; // sub x5, x1, x2

        // 7. and x6, x1, x2 (R-type)
        cpu.fetch_stage.imem.mem[7] = 32'b0000000_00010_00001_111_00110_0110011; // and x6, x1, x2

        // 5. add x4, x1, x2 (R-type)
        cpu.fetch_stage.imem.mem[8] = 32'b0000000_00010_00001_000_00100_0110011; // add x4, x1, x2


        // 8. or x7, x1, x2 (R-type)
        cpu.fetch_stage.imem.mem[9] = 32'b0000000_00010_00001_110_00111_0110011; // or x7, x1, x2

        // 9. xor x8, x1, x2 (R-type)
        cpu.fetch_stage.imem.mem[10] = 32'b0000000_00010_00001_100_01000_0110011; // xor x8, x1, x2

        // 10. sll x9, x1, x2 (R-type)
        cpu.fetch_stage.imem.mem[11] = 32'b0000000_00010_00001_001_01001_0110011; // sll x9, x1, x2

        // 11. srl x10, x1, x2 (R-type)
        cpu.fetch_stage.imem.mem[12] = 32'b0000000_00010_00101_101_01010_0110011; // srl x10, x5, x2

        // 12. sra x11, x1, x2 (R-type)
        cpu.fetch_stage.imem.mem[13] = 32'b0100000_00010_00101_101_01011_0110011; // sra x11, x5, x2

        // 13. slt x12, x1, x2 (R-type)
        cpu.fetch_stage.imem.mem[14] = 32'b0000000_00010_00001_010_01100_0110011; // slt x12, x1, x2

        // 14. sltu x13, x1, x2 (R-type)
        cpu.fetch_stage.imem.mem[15] = 32'b0000000_00010_00001_011_01101_0110011; // sltu x13, x1, x2

        // 15. jal x0, 0 (infinite loop)
        cpu.fetch_stage.imem.mem[16] = 32'b0000000_00000_00000_000_00000_1101111; // jal x0, 0 (infinite loop)
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