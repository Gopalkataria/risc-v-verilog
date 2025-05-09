`timescale 1ns / 1ps

module RISC_V_CPU_Test;
    reg clk;
    reg reset;
    
   
    riscv_processor cpu (
        .clk(clk),
        .rst(reset)
    );
    
   
    initial begin
        $dumpfile("memory_test.vcd");
        $dumpvars(0, RISC_V_CPU_Test);
    end
    
   
    initial begin
        clk = 0;
        forever #1 clk = ~clk;
    end
    
    integer i;
    initial begin
       
        reset = 1;
        #6 reset = 0;
        
       
        for (i = 0; i < 1024; i = i + 1) begin
            cpu.fetch_stage.imem.mem[i] = 32'h00000000;
        end
        
       
        cpu.memory_stage.dmem.mem[0]  = 64'h1234567812345678; // Word at address 0
        cpu.memory_stage.dmem.mem[1]  = 64'h9ABCDEF09ABCDEF0; // Word at address 4
        cpu.memory_stage.dmem.mem[2]  = 64'h1122334411223344; // Word at address 8
        cpu.memory_stage.dmem.mem[3]  = 64'haaaaaaaaaaaaaaaa; // Word at address 12
        cpu.memory_stage.dmem.mem[4]  = 64'hbbbbbbbbbbbbbbbb; // Word at address 16
        cpu.memory_stage.dmem.mem[5]  = 64'hcccccccccccccccc; // Word at address 20

       
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

    end
    
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
        $monitor("| %4t ns | PC  %0d | INST  %b | RES %08h | B ?  %b    | JUMP %08h |\n| Register Values: x1=0x%08h x2=0x%08h x3=0x%08h |\n| Register Values: x4=0x%08h x5=0x%08h x6=0x%08h |\n | Memory Values: [0]=0x%08h [4]=0x%08h [8]=0x%08h \n \t  [12]=0x%08h [16]=0x%08h [20]=0x%08h |\n",
            $time, 
            cpu.fetch_stage.pc, 
            cpu.if_id_register.instruction_out, 
            cpu.execute_stage.alu_result, 
            cpu.execute_stage.branch_taken, // Branch 
            cpu.execute_stage.jump_target, // Jump target 
            cpu.decode_stage.reg_file.registers[1], // x1 
            cpu.decode_stage.reg_file.registers[2], // x2 
            cpu.decode_stage.reg_file.registers[3], // x3 
            cpu.decode_stage.reg_file.registers[4], // x1 
            cpu.decode_stage.reg_file.registers[5], // x2 
            cpu.decode_stage.reg_file.registers[6], // x3 
            cpu.memory_stage.dmem.mem[0], //  0
            cpu.memory_stage.dmem.mem[1], //  4
            cpu.memory_stage.dmem.mem[2], //  8
            cpu.memory_stage.dmem.mem[3], //  12
            cpu.memory_stage.dmem.mem[4], //  16
            cpu.memory_stage.dmem.mem[5]  //  20
        );
    end
endmodule