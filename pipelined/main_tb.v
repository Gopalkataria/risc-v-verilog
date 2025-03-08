module RISC_V_Pipelined_CPU_TB;
    reg clk;
    reg reset;
    
    // Instantiate the RISC-V CPU
    riscv_processor cpu (
        .clk(clk),
        .rst(reset)
    );
    
    // GTKWave Dump
    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, RISC_V_Pipelined_CPU_TB);
    end
    
    // Clock Generation (10ns period)
    initial begin
        clk = 0;
        forever #1 clk = ~clk;
    end
    
    // Reset and Initialization
        integer i;
    initial begin
        // Initialize memory with all zeros first
        for (i = 0; i < 1024; i = i + 1) begin
            cpu.fetch_stage.imem.mem[i] = 32'h00000000;
        end
        
        // Apply reset
        reset = 1;
        #10 reset = 0; // Deassert reset after 10 time units
      // Writing instructions to memory
cpu.fetch_stage.imem.mem[0] = 32'h00500093; // addi x1, x0, 5 (x1 = 5)
cpu.fetch_stage.imem.mem[1] = 32'h00A00113; // addi x2, x0, 10 (x2 = 10)

// Padding with 5 NOPs
cpu.fetch_stage.imem.mem[2] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[3] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[4] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[5] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[6] = 32'h00000013; // nop

cpu.fetch_stage.imem.mem[7] = 32'h002081B3; // add x3, x1, x2 (x3 = x1 + x2 = 15)
cpu.fetch_stage.imem.mem[8] = 32'h40208233; // sub x4, x1, x2 (x4 = x1 - x2 = -5)

// Padding with 5 NOPs
cpu.fetch_stage.imem.mem[9] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[10] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[11] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[12] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[13] = 32'h00000013; // nop

cpu.fetch_stage.imem.mem[14] = 32'h0020F2B3; // and x5, x1, x2 (x5 = x1 & x2 = 0)
cpu.fetch_stage.imem.mem[15] = 32'h0020E333; // or x6, x1, x2 (x6 = x1 | x2 = 15)

// Padding with 5 NOPs
cpu.fetch_stage.imem.mem[16] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[17] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[18] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[19] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[20] = 32'h00000013; // nop

cpu.fetch_stage.imem.mem[21] = 32'h0020C3B3; // xor x7, x1, x2 (x7 = x1 ^ x2 = 15)
cpu.fetch_stage.imem.mem[22] = 32'h00109433; // sll x8, x1, x1 (x8 = x1 << x1 = 160)

// Padding with 5 NOPs
cpu.fetch_stage.imem.mem[23] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[24] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[25] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[26] = 32'h00000013; // nop
cpu.fetch_stage.imem.mem[27] = 32'h00000013; // nop

cpu.fetch_stage.imem.mem[28] = 32'h0020D4B3; // srl x9, x1, x2 (x9 = x1 >> x2 = 0)
cpu.fetch_stage.imem.mem[29] = 32'h4020D533; // sra x10, x1, x2 (x10 = x1 >>> x2 = 0)


    end 
    
    // Termination
    initial begin
        #50;
        $finish;
    end
    
    // Monitoring key signals
    initial begin
        $display("Time | PC       | Instr    | ALUResult        | Reg1           | Reg2           | Reg3           | Reg4           | Branch | Jump");
        $monitor("%3t | %h | %h | %h | %h | %h | %h | %h | %b | %h",
            $time,
            cpu.fetch_stage.pc,
            cpu.if_id_register.instruction_out,
            cpu.execute_stage.alu_result,
            cpu.decode_stage.reg_file.registers[1],
            cpu.decode_stage.reg_file.registers[2],
            cpu.decode_stage.reg_file.registers[3],
            cpu.decode_stage.reg_file.registers[4],
            cpu.execute_stage.branch_taken,
            cpu.execute_stage.jump_target);
    end
endmodule