# Custom 8-Bit CPU Design using Verilog HDL

## Project Overview

This project focuses on the design and development of a fully custom 8-bit Central Processing Unit implemented entirely in Verilog HDL. The CPU follows a modular approach, separating datapath and control logic to clearly demonstrate fundamental digital system principles such as instruction decoding, ALU processing, control sequencing, branching logic, and memory interfacing at the RTL level.

The design provides a practical representation of how a basic processor operates internally, from instruction fetch to result writeback.

---

## Technology Stack

* **HDL:** Verilog
* **Simulation Tools:** ModelSim, Icarus Verilog, GTKWave
* **Design Methodology:** Behavioral + Structural RTL
* **Architecture Style:** Harvard-based, word-addressable CPU
* **Instruction Size:** 24-bit

---

## CPU Instruction Format (24-bit)

| Bit Range | Field     | Description                    |
| --------- | --------- | ------------------------------ |
| 23–21     | Opcode    | Specifies the instruction type |
| 20–18     | Rd        | Destination register address   |
| 17–15     | Rs1       | Source register 1              |
| 14–12     | Rs2       | Source register 2              |
| 11–3      | Immediate | 9-bit immediate value          |
| 2–0       | Mode Flag | Controls branch/jump behavior  |

---

## Major System Modules

| Module                       | Purpose                                            |
| ---------------------------- | -------------------------------------------------- |
| cpu_top.v                    | Top-level integration of control and datapath      |
| cpu_datapath.v               | Performs ALU operations and data routing           |
| controller.v                 | FSM-based control logic for instruction sequencing |
| ALU.v                        | Executes arithmetic and logical functions          |
| Reg_File.v                   | 8-register × 8-bit register bank                   |
| Memory.v                     | 256 × 8-bit memory for data storage                |
| PC, MUX_pc, Adder, IR, IRmem | Manage program flow and instruction fetching       |

---

## Supported Instruction Set

| Opcode | Instruction | Function              |     |
| ------ | ----------- | --------------------- | --- |
| 000    | ADD         | Rd = Rs1 + Rs2        |     |
| 001    | SUB         | Rd = Rs1 - Rs2        |     |
| 010    | AND         | Rd = Rs1 & Rs2        |     |
| 011    | OR          | Rd = Rs1              | Rs2 |
| 100    | LOAD        | Rd = Memory[address]  |     |
| 101    | STORE       | Memory[address] = Rs2 |     |
| 110    | BRANCH      | Conditional branching |     |
| 111    | JUMP        | Direct program jump   |     |

### Branch Control Modes

* 000 → BEQ (Branch if Equal)
* 001 → BNE (Branch if Not Equal)
* 010 → BLT (Branch if Less Than)
* 011 → BGT (Branch if Greater Than)

---

## Control Unit Operation (FSM)

The CPU follows a four-stage control sequence managed by an FSM:

1. **Fetch** – Retrieves instruction from instruction memory
2. **Decode/Execute** – Decodes opcode and performs ALU operations
3. **Memory Access** – Handles data read/write when required
4. **Writeback** – Stores result into destination register

---

## Key Features

* Clear separation of datapath and control structures
* Functional 8-bit ALU with flag generation (Zero, Carry, Negative, Overflow)
* Supports load, store, branch, and jump instructions
* Synchronous memory operations
* Scalable instruction architecture for future expansion

---

## Simulation & Verification

Run simulation using Icarus Verilog:

```
iverilog -o cpu_sim cpu_top.v
vvp cpu_sim
```

View waveform results:

```
gtkwave dump.vcd
```

Instruction memory content can be modified to test custom programs and instruction sequences.

---

## Learning Outcomes

* Developed a complete RTL CPU from scratch
* Implemented FSM-controlled instruction execution
* Designed ALU with flag computation
* Understood instruction decoding and pipeline flow
* Integrated register file and memory subsystems

---

## Future Enhancements

* Introduce full 5-stage pipeline (Fetch–Decode–Execute–Memory–Writeback)
* Add multiplication and division support
* Implement hazard detection and forwarding logic
* Deploy on FPGA for hardware-level testing

---

## Author

**Deepankar Majee**
Electronics and Communication Engineering Student
Interests: Digital System Design, VLSI Architecture, and Embedded Processing
