module RISC_V_Pipelined_CPU_TB;
    reg clk;
    reg reset;
    
    // Instantiate the CPU
    RISC_V_Pipelined_CPU cpu (
        .clk(clk),
        .reset(reset)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns clock period (100MHz)
    end
    
    // Initialize instruction memory with a test program
    initial begin
        // Sample RISC-V program (can be modified as needed)
        // These are just placeholders - you would replace with actual instructions
        cpu.fetch_stage.instr_mem.instr_mem[0] = 32'h00500093; // addi x1, x0, 5
        cpu.fetch_stage.instr_mem.instr_mem[1] = 32'h00A00113; // addi x2, x0, 10
        cpu.fetch_stage.instr_mem.instr_mem[2] = 32'h002081B3; // add x3, x1, x2
        cpu.fetch_stage.instr_mem.instr_mem[3] = 32'h0041A233; // slt x4, x3, x4
        cpu.fetch_stage.instr_mem.instr_mem[4] = 32'h00208463; // beq x1, x2, 8 (branch if x1 == x2)
        cpu.fetch_stage.instr_mem.instr_mem[5] = 32'h00100073; // ebreak
        cpu.fetch_stage.instr_mem.instr_mem[6] = 32'h004000EF; // jal x1, 4 (jump and link)
        cpu.fetch_stage.instr_mem.instr_mem[7] = 32'h00008067; // jalr x0, 0(x1) (jump and link register)
    end
    
    // Test sequence
    initial begin
        // Initialize
        reset = 1;
        #20;
        reset = 0;
        
        // Run for some cycles to see the execution
        #200;
        
        $finish;
    end
    
    // Monitoring
    initial begin
        $monitor("Time=%0t, PC=%h, Instr=%h, ALUResult=%h, Reg1=%h, Reg2=%h, Reg3=%h, BranchTaken=%b, JumpTaken=%b", 
                 $time, cpu.PC_Out, cpu.Instr, cpu.ALUResult, cpu.reg_file.registers[1], cpu.reg_file.registers[2], cpu.reg_file.registers[3], cpu.branch_taken, cpu.jump_taken);
    end
endmodule