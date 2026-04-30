# Simplified CPU Datapath Design (Verilog)

## Overview

This project focuses on the design and implementation of a simplified CPU datapath using Verilog HDL. The system includes core components such as registers, register files, an Arithmetic Logic Unit (ALU), and control logic, all operating under a unified clock signal.

The goal of the project is to model low-level hardware behavior and understand how fundamental processor components interact at the bit level.

---

## Key Features

### 1. Register Design

* Implemented 8-bit and 16-bit registers
* Supported operations:
  * Load
  * Clear
  * Increment / Decrement
  * Hold (no operation)
* Controlled via function select (FunSel) and enable signals

### 2. Register File

* Designed a multi-register structure with selectable outputs
* Supports:

  * Simultaneous read operations
  * Selective write/update via control signals
* Includes both:

  * General-purpose registers
  * Address registers (PC, AR, SP)

### 3. Instruction Register (IR)

* 16-bit instruction register with partial loading capability
* Supports loading high and low bytes separately
* Enables flexible instruction handling

### 4. Arithmetic Logic Unit (ALU)

* 8-bit ALU supporting multiple operations:

  * Arithmetic: Addition, Subtraction
  * Logical: AND, OR, XOR, NOT
  * Shift operations:

    * Logical (LSL, LSR)
    * Arithmetic (ASR, ASL)
    * Circular (CSR, CSL)
* Status flags:

  * Zero (Z)
  * Carry (C)
  * Negative (N)
  * Overflow (O)

### 5. System Integration

* Integrated all components into a unified datapath
* Designed multiplexers to control data flow between units
* Implemented a synchronous system using a single clock

---

## Technical Highlights

* Bit-level hardware modeling using Verilog
* Control signal design and state-based behavior
* Modular design approach for reusability
* Simulation-driven verification for each module
* Understanding of datapath and control unit interaction

---

## What I Learned

* How processor components interact at a low level
* Difference between combinational and sequential logic
* Importance of control signals in hardware design
* Handling edge cases such as overflow, carry, and timing
* Designing fundemental hardware systems

---

## Possible Improvements

* Adding pipelining support
* Implementing a control unit with instruction decoding
* Expanding instruction set
* Integrating memory hierarchy (cache simulation)

---

## Tools & Technologies

* Verilog HDL
* Vivado

---

## Notes

This project reflects a foundational understanding of computer architecture concepts and low-level system design, which are directly applicable to embedded and real-time systems development.
