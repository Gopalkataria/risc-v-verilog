module RISC_V_Pipelined_CPU_TB;
    reg clk;
    reg reset;
    
    // Instantiate CPU
    RISC_V_Pipelined_CPU cpu (
        .clk(clk),
        .reset(reset)
    );
    
    // GTKWave Dump
    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, RISC_V_Pipelined_CPU_TB);
    end
    
    // Clock Generation (10ns period)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // Instruction Memory Initialization
    initial begin
        reset = 1;
        #10 reset = 0;  // Deassert reset after 10 time units

        // Wait for reset to settle
        #5;
        
        
            cpu.fetch_stage.instr_mem.instr_mem[0] = 32'h00500093; // addi x1, x0, 5
            cpu.fetch_stage.instr_mem.instr_mem[1] = 32'h00A00113; // addi x2, x0, 10
            cpu.fetch_stage.instr_mem.instr_mem[2] = 32'h002081B3; // add x3, x1, x2
            cpu.fetch_stage.instr_mem.instr_mem[3] = 32'h0041A233; // sw x4, 0(x3)
            cpu.fetch_stage.instr_mem.instr_mem[4] = 32'h00208463; // beq x1, x2, label
            cpu.fetch_stage.instr_mem.instr_mem[6] = 32'h004000EF; // jal x1, label
            cpu.fetch_stage.instr_mem.instr_mem[7] = 32'h00008067; // ret
            cpu.fetch_stage.instr_mem.instr_mem[5] = 32'h00100073; // ecall
        
    end

    // Termination
    initial begin
        #200;
        $finish;
    end

    // Monitoring key signals
    initial begin
        $display("Time| PC       | Instr    | ALUResult| Reg1     | Reg2     | Reg3     | Reg4     | Reg4     |Branch     | Jump");

        $monitor("%3t | %h | %h | %h | %h | %h | %h | %h | %h | %b \t| %b", 
                 $time, cpu.PC_Out, cpu.Instr, cpu.ALUResult, 
                 cpu.decode_stage.registers[1], 
                 cpu.decode_stage.registers[2], 
                 cpu.decode_stage.registers[3], 
                    cpu.decode_stage.registers[4],
                    cpu.decode_stage.registers[5],
                 cpu.BranchTaken, cpu.Jump);
    end

endmodule
