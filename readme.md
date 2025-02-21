# IPA Spring'25 Project 

Contributers : 

| Name               | Roll Number |
| ------------------ | ----------- |
| Devansh Vershney   | 2023102042  |
| Gopal Kataria      | 2023112006  |
| Shaikh Haris Jamal | 2023112007  |

# RISC-V Sequential Implementation

This project is a sequential implementation of the RISC-V architecture. The implementation is divided into five stages, each represented by its respective file.

## Stages

1. **Instruction Fetch (IF)**
    - File: `instruction_fetch.v`
    - Description: This stage is responsible for fetching the instruction from memory using the program counter (PC).

2. **Instruction Decode (ID)**
    - File: `instruction_decode.v`
    - Description: This stage decodes the fetched instruction and reads the necessary registers.

3. **Execution (EX)**
    - File: `execution.v`
    - Description: This stage performs arithmetic and logical operations based on the decoded instruction.

4. **Memory Access (MEM)**
    - File: `memory_access.v`
    - Description: This stage handles memory read and write operations.

5. **Write Back (WB)**
    - File: `write_back.v`
    - Description: This stage writes the result of the execution or memory access back to the register file.

