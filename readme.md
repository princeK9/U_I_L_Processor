
# **U_I_L_P**


The **UART-Integrated LIPSI Processor (U_I_L_P)** is a compact and efficient 8-bit processor core designed for educational and experimental purposes with a limited instruction set,coded entirely in **Verilog**,implemented on  **FPGA** . It integrates a UART interface for program loading and features a simplified version of the LIPSI (Little Instruction Processor with Simple Implementation) architecture. This processor follows harvard architecture with 2 memory blocks one for instruction and other for data.


---


## **Features**

- **Simple 8-bit Processor**
- **UART receiver** for program loading (Hex format)
- **Follows harvard architecture :: Instruction Memory (IMEM)** and **Data Memory (DMEM)**
- **Accumulator (ACC) register**
- **Basic ALU operations**:
    - Add, Subtract, Add with Carry, Subtract with Borrow
    - AND, OR, XOR, Move/Load
    - Shift/Rotate operations: shift / rotate + left / right + with / without carry

- **Conditional and Unconditional Jumps**
- **Simple Call/Return mechanism**:
    - Stores return address in DMEM, jumps via ACC
- **HALT instruction** to signal program completion
- **Flags**: Carry (C) and Zero (Z)
- **Implemented on FPGA** 
- **Waiting LED output** while receiving UART data
- **RX Done signal** after UART loading timeout

---

## **Project Structure**

The project is organized into several Verilog modules, connected together in the `top_level` module:

### **Top-Level Module**
- **`top_level.v`**: Connects all sub-modules and handles system-level functionality.

### **UART to Instruction Memory**
- **`uart_to_imem.v`**: Handles UART reception, ASCII to Hex conversion, and writing to Instruction Memory.
    - **`uart_rx.v`**: Standard UART receiver module.
    - **`imem.v`**: Instruction Memory module.

### **Finite State Machine (FSM)**
- **`fsm.v`**: Controls the CPU's fetch-decode-execute cycle.
    - **`pc.v`**: Program Counter module.
    - **`acc.v`**: Accumulator register module.
    - **`dmem.v`**: Data Memory module.
    - **`alu.v`**: Arithmetic Logic Unit module.

### **Display Driver**
- **`display_driver.v`**: Controls multiplexing and decoding for the 4-digit 7-segment display.
    - **`seven_seg_decoder.v`**: Decodes a 4-bit binary value into 7-segment display patterns.

---

## **Module Descriptions**

### **`top_level`**
The main wrapper module that connects all components. It handles:
- **Clock and Reset**
- **UART Input (`rx`)**
- **7-Segment Display Outputs (`an`, `seg`)**
- **Status LEDs (`waiting_led`, `rx_done`, `done`)**

### **`uart_to_imem`**
Interfaces the UART receiver to the instruction memory (`imem`):
- Converts ASCII Hex characters to 8-bit instructions.
- Writes instructions sequentially into `imem`.
- Signals `rx_done` after a timeout to start program execution.

### **`fsm`**
Implements the CPU's control logic as a Finite State Machine:
- **FETCH-DECODE-EXECUTE Cycle**:
    - Fetches instructions from `imem`.
    - Decodes and executes instructions using the `alu`, `acc`, and `dmem`.
- Handles program and the data flow, jumps, and HALT.

### **`display_driver`**
Drives the 4-digit multiplexed 7-segment display:
- Displays the **last received byte** and **final accumulator value** in hexadecimal format.

---

### **Instruction Set**

For a detailed description of the instruction set, please refer to the [ISA.txt](./ISA.txt) file available in the repository.

---


## **How it Works**

1. **Reset**: Initializes the system.
2. **Program Loading**: Loads the program via UART in ASCII Hex format.
3. **Execution**: Executes instructions sequentially from `imem`.
4. **Completion**: HALT instruction stops execution and displays the Accumulator as result.

---

## **Requirements**

- **Verilog Simulator**: Icarus Verilog, ModelSim/ Vivado Simulator
- **UART-TOOLs**: Tera-Term
- **FPGA Tools**: Xilinx Vivado/ Intel Quartus
- **Target FPGA Board**: i/o
    - Clock input (`clk`)
    - Reset button (`reset`)
    - UART RX pin (`rx`)
    - 4-digit multiplexed 7-segment display (`an`, `seg`)
    - LEDs (`waiting_led`, `rx_done`, `done`)

---


## **Usage**

1. Synthesize and implement the Verilog code.
2. Connect a UART transmitter to the FPGA.
3. Reset the FPGA.
4. Send program bytes in ASCII Hex format through Tera-Term software.
5. Wait for the timeout/rx_done to start execution.
6. Observe the Accumulator on the 7-segment display.


---

## **Potential Enhancements**

- Add more instructions and addressing modes.
- Implement a stack for subroutine calls.
- Decrease the LUTs used for FPGA implementation for optimisation of this model.

---

## **Steps to Use**

1. **Clone the Repository**:
    ```bash
    git clone <repository_url>
    cd <repository_folder>
    ```

2. **Locate the Code**:
    - Navigate to the `phase_3` folder.
    - Use the `lipsi.v` code for implementation.

3. **FPGA Implementation**:
    - If implementing on an FPGA, use the provided `.xdc` file to define input and output ports.
    - Ensure the ports are correctly named according to your FPGA board's pin configuration.

4. **Clock Configuration**:
    - The current code is designed for a **100 MHz clock**.
    - If your FPGA uses a different clock frequency, update the clock settings in the `uart_rx.v` module.

5. **Create a Project**:
    - Use software like **Vivado** or **Multisim** to create a new project.
    - Add all the required Verilog files and constraints.
    - Synthesize, implement, and program the FPGA.

6. **Program Execution**:
    - Connect a UART transmitter to the FPGA.
    - Reset the FPGA and send program bytes in ASCII Hex format.
    - Observe the results on the 7-segment display.
