# Sequential Implementation
Use this folder to house the implementation of the sequential RISC V processor
- put all module files into a folder called `src` and have the main wrapper file as `main.v` 

# running instructions 
```sh
iverilog -o main.out main_tb.v ./main.v ./src/*.v ; vvp main.out
```