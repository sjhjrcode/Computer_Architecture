
//////////////////////////////////////////
// wally-shared.vh
//
// Written: david_harris@hmc.edu 7 June 2021
//
// Purpose: Shared and default configuration values common to all designs
//
// A component of the Wally configurable RISC-V project.
//
// Copyright (C) 2021 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////

// constants defining different privilege modes
// defined in Table 1.1 of the privileged spec
`define M_MODE (2'b11)
`define S_MODE (2'b01)
`define U_MODE (2'b00)

// Virtual Memory Constants
`define VPN_SEGMENT_BITS (`XLEN == 32 ? 10 : 9)
`define VPN_BITS (`XLEN==32 ? (2*`VPN_SEGMENT_BITS) : (4*`VPN_SEGMENT_BITS))
`define PPN_BITS (`XLEN==32 ? 22 : 44)
`define PA_BITS (`XLEN==32 ? 34 : 56)
`define SVMODE_BITS (`XLEN==32 ? 1 : 4)
`define ASID_BASE (`XLEN==32 ? 22 : 44)
`define ASID_BITS (`XLEN==32 ? 9 : 16)

// constants to check SATP_MODE against
// defined in Table 4.3 of the privileged spec
`define NO_TRANSLATE 0
`define SV32 1
`define SV39 8
`define SV48 9

// macros to define supported modes
`define A_SUPPORTED ((`MISA >> 0) % 2 == 1)
`define B_SUPPORTED ((`ZBA_SUPPORTED | `ZBB_SUPPORTED | `ZBC_SUPPORTED | `ZBS_SUPPORTED)) // not based on MISA
`define C_SUPPORTED ((`MISA >> 2) % 2 == 1)
`define D_SUPPORTED ((`MISA >> 3) % 2 == 1)
`define E_SUPPORTED ((`MISA >> 4) % 2 == 1)
`define F_SUPPORTED ((`MISA >> 5) % 2 == 1)
`define I_SUPPORTED ((`MISA >> 8) % 2 == 1)
`define M_SUPPORTED ((`MISA >> 12) % 2 == 1)
`define Q_SUPPORTED ((`MISA >> 16) % 2 == 1)
`define S_SUPPORTED ((`MISA >> 18) % 2 == 1)
`define U_SUPPORTED ((`MISA >> 20) % 2 == 1)
// N-mode user-level interrupts are depricated per Andrew Waterman 1/13/21

// logarithm of XLEN, used for number of index bits to select
`define LOG_XLEN (`XLEN == 32 ? 5 : 6)

// Number of 64 bit PMP Configuration Register entries (or pairs of 32 bit entries)
`define PMPCFG_ENTRIES (`PMP_ENTRIES/8)

// Floating point constants for Quad, Double, Single, and Half precisions
`define Q_LEN 32'd128
`define Q_NE 32'd15
`define Q_NF 32'd112
`define Q_BIAS 32'd16383
`define Q_FMT 2'd3
`define D_LEN 32'd64
`define D_NE 32'd11
`define D_NF 32'd52
`define D_BIAS 32'd1023
`define D_FMT 2'd1
`define S_LEN 32'd32
`define S_NE 32'd8
`define S_NF 32'd23
`define S_BIAS 32'd127
`define S_FMT 2'd0
`define H_LEN 32'd16
`define H_NE 32'd5
`define H_NF 32'd10
`define H_BIAS 32'd15
`define H_FMT 2'd2

// Floating point length FLEN and number of exponent (NE) and fraction (NF) bits
`define FLEN (`Q_SUPPORTED ? `Q_LEN  : `D_SUPPORTED ? `D_LEN  : `S_LEN)
`define NE   (`Q_SUPPORTED ? `Q_NE   : `D_SUPPORTED ? `D_NE   : `S_NE)
`define NF   (`Q_SUPPORTED ? `Q_NF   : `D_SUPPORTED ? `D_NF   : `S_NF)
`define FMT  (`Q_SUPPORTED ? 2'd3    : `D_SUPPORTED ? 2'd1    : 2'd0)
`define BIAS (`Q_SUPPORTED ? `Q_BIAS : `D_SUPPORTED ? `D_BIAS : `S_BIAS)
/* Delete once tested dh 10/10/22

`define FLEN (`Q_SUPPORTED ? `Q_LEN  : `D_SUPPORTED ? `D_LEN  : `F_SUPPORTED ? `S_LEN  : `H_LEN)
`define NE   (`Q_SUPPORTED ? `Q_NE   : `D_SUPPORTED ? `D_NE   : `F_SUPPORTED ? `S_NE   : `H_NE)
`define NF   (`Q_SUPPORTED ? `Q_NF   : `D_SUPPORTED ? `D_NF   : `F_SUPPORTED ? `S_NF   : `H_NF) 
`define FMT  (`Q_SUPPORTED ? 2'd3       : `D_SUPPORTED ? 2'd1       : `F_SUPPORTED ? 2'd0       : 2'd2)
`define BIAS (`Q_SUPPORTED ? `Q_BIAS : `D_SUPPORTED ? `D_BIAS : `F_SUPPORTED ? `S_BIAS : `H_BIAS)*/

// Floating point constants needed for FPU paramerterization
`define FPSIZES ((32)'(`Q_SUPPORTED)+(32)'(`D_SUPPORTED)+(32)'(`F_SUPPORTED)+(32)'(`ZFH_SUPPORTED))
`define FMTBITS ((32)'(`FPSIZES>=3)+1)
`define LEN1  ((`D_SUPPORTED & (`FLEN != `D_LEN)) ? `D_LEN  : (`F_SUPPORTED & (`FLEN != `S_LEN)) ? `S_LEN  : `H_LEN)
`define NE1   ((`D_SUPPORTED & (`FLEN != `D_LEN)) ? `D_NE   : (`F_SUPPORTED & (`FLEN != `S_LEN)) ? `S_NE   : `H_NE)
`define NF1   ((`D_SUPPORTED & (`FLEN != `D_LEN)) ? `D_NF   : (`F_SUPPORTED & (`FLEN != `S_LEN)) ? `S_NF   : `H_NF)
`define FMT1  ((`D_SUPPORTED & (`FLEN != `D_LEN)) ? 2'd1    : (`F_SUPPORTED & (`FLEN != `S_LEN)) ? 2'd0    : 2'd2)
`define BIAS1 ((`D_SUPPORTED & (`FLEN != `D_LEN)) ? `D_BIAS : (`F_SUPPORTED & (`FLEN != `S_LEN)) ? `S_BIAS : `H_BIAS)
`define LEN2  ((`F_SUPPORTED & (`LEN1 != `S_LEN)) ? `S_LEN  : `H_LEN)
`define NE2   ((`F_SUPPORTED & (`LEN1 != `S_LEN)) ? `S_NE   : `H_NE)
`define NF2   ((`F_SUPPORTED & (`LEN1 != `S_LEN)) ? `S_NF   : `H_NF)
`define FMT2  ((`F_SUPPORTED & (`LEN1 != `S_LEN)) ? 2'd0    : 2'd2)
`define BIAS2 ((`F_SUPPORTED & (`LEN1 != `S_LEN)) ? `S_BIAS : `H_BIAS)

// largest length in IEU/FPU
`define CVTLEN ((`NF<`XLEN) ? (`XLEN) : (`NF))
`define LLEN (($unsigned(`FLEN)<$unsigned(`XLEN)) ? ($unsigned(`XLEN)) : ($unsigned(`FLEN)))
`define LOGCVTLEN $unsigned($clog2(`CVTLEN+1))
`define NORMSHIFTSZ (((`CVTLEN+`NF+1)>(`DIVb + 1 +`NF+1) & (`CVTLEN+`NF+1)>(3*`NF+6)) ? (`CVTLEN+`NF+1) : ((`DIVb + 1 +`NF+1) > (3*`NF+6) ? (`DIVb + 1 +`NF+1) : (3*`NF+6)))
`define LOGNORMSHIFTSZ ($clog2(`NORMSHIFTSZ))
`define CORRSHIFTSZ (((`CVTLEN+`NF+1)>(`DIVb + 1 +`NF+1) & (`CVTLEN+`NF+1)>(3*`NF+6)) ? (`CVTLEN+`NF+1) : ((`DIVN+1+`NF) > (3*`NF+4) ? (`DIVN+1+`NF) : (3*`NF+4)))

// division constants

`define DIVN        (((`NF<`XLEN) & `IDIV_ON_FPU) ? `XLEN : `NF+2) // standard length of input
`define LOGR        ($clog2(`RADIX))            // r = log(R)
`define RK          (`LOGR*`DIVCOPIES)          // r*k used for intdiv preproc
`define LOGRK       ($clog2(`RK))               // log2(r*k)
`define FPDUR       ((`DIVN+1+(`LOGR*`DIVCOPIES))/(`LOGR*`DIVCOPIES)+(`RADIX/4))
`define DURLEN      ($clog2(`FPDUR+1))
`define DIVb        (`FPDUR*`LOGR*`DIVCOPIES-1) // canonical fdiv size (b)
`define DIVBLEN     ($clog2(`DIVb+1)-1)
`define DIVa        (`DIVb+1-`XLEN)             // used for idiv on fpu

// Disable spurious Verilator warnings

/* verilator lint_off STMTDLY */
/* verilator lint_off ASSIGNDLY */
/* verilator lint_off PINCONNECTEMPTY */


///////////////////////////////////////////
// alu.sv
//
// Written: David_Harris@hmc.edu, Sarah.Harris@unlv.edu, kekim@hmc.edu
// Created: 9 January 2021
// Modified: 3 March 2023
//
// Purpose: RISC-V Arithmetic/Logic Unit
//
// Documentation: RISC-V System on Chip Design Chapter 4 (Figure 4.4)
// 
// A component of the CORE-V-WALLY configurable RISC-V project.
// 
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////


`define FPGA 0
`define QEMU 0

// RV32 or RV64: XLEN = 32 or 64
`define XLEN 32

// IEEE 754 compliance
`define IEEE754 0

// E
`define MISA (32'h00000010) 
`define ZICSR_SUPPORTED 0
`define ZIFENCEI_SUPPORTED 0
`define COUNTERS 0
`define ZICOUNTERS_SUPPORTED 0
`define ZFH_SUPPORTED 0
`define SSTC_SUPPORTED 0

// LSU microarchitectural Features
`define BUS_SUPPORTED 1
`define DCACHE_SUPPORTED 0
`define ICACHE_SUPPORTED 0
`define VIRTMEM_SUPPORTED 0
`define VECTORED_INTERRUPTS_SUPPORTED 0 
`define BIGENDIAN_SUPPORTED 0

// TLB configuration.  Entries should be a power of 2
`define ITLB_ENTRIES 0
`define DTLB_ENTRIES 0

// Cache configuration.  Sizes should be a power of two
// typical configuration 4 ways, 4096 bytes per way, 256 bit or more lines
`define DCACHE_NUMWAYS 4
`define DCACHE_WAYSIZEINBYTES 4096
`define DCACHE_LINELENINBITS 512
`define ICACHE_NUMWAYS 4
`define ICACHE_WAYSIZEINBYTES 4096
`define ICACHE_LINELENINBITS 512

// Integer Divider Configuration
// IDIV_BITSPERCYCLE must be 1, 2, or 4
`define IDIV_BITSPERCYCLE 1
`define IDIV_ON_FPU 0

// Legal number of PMP entries are 0, 16, or 64
`define PMP_ENTRIES 0

// Address space
`define RESET_VECTOR 32'h80000000

// WFI Timeout Wait
`define WFI_TIMEOUT_BIT 16

// Peripheral Addresses
// Peripheral memory space extends from BASE to BASE+RANGE
// Range should be a thermometer code with 0's in the upper bits and 1s in the lower bits
`define DTIM_SUPPORTED 1'b0
`define DTIM_BASE       34'h80000000
`define DTIM_RANGE      34'h007FFFFF
`define IROM_SUPPORTED 1'b0
`define IROM_BASE       34'h80000000
`define IROM_RANGE      34'h007FFFFF
`define BOOTROM_SUPPORTED 1'b1
`define BOOTROM_BASE   34'h00001000 
`define BOOTROM_RANGE  34'h00000FFF
`define UNCORE_RAM_SUPPORTED 1'b1
`define UNCORE_RAM_BASE       34'h80000000
`define UNCORE_RAM_RANGE      34'h07FFFFFF
`define EXT_MEM_SUPPORTED 1'b0
`define EXT_MEM_BASE       34'h80000000
`define EXT_MEM_RANGE      34'h07FFFFFF
`define CLINT_SUPPORTED 1'b0
`define CLINT_BASE  34'h02000000
`define CLINT_RANGE 34'h0000FFFF
`define GPIO_SUPPORTED 1'b0
`define GPIO_BASE   34'h10060000
`define GPIO_RANGE  34'h000000FF
`define UART_SUPPORTED 1'b0
`define UART_BASE   34'h10000000
`define UART_RANGE  34'h00000007
`define PLIC_SUPPORTED 1'b0
`define PLIC_BASE   34'h0C000000
`define PLIC_RANGE  34'h03FFFFFF
`define SDC_SUPPORTED 1'b0
`define SDC_BASE   34'h00012100
`define SDC_RANGE  34'h0000001F

// Bus Interface width
`define AHBW 32

// Test modes

// Tie GPIO outputs back to inputs
`define GPIO_LOOPBACK_TEST 1

// Hardware configuration
`define UART_PRESCALE 1

// Interrupt configuration
`define PLIC_NUM_SRC 10 
// comment out the following if >=32 sources
`define PLIC_NUM_SRC_LT_32
`define PLIC_GPIO_ID 3
`define PLIC_UART_ID 10

`define BPRED_SUPPORTED 0
`define BPRED_TYPE "BP_GSHARE" // BP_GSHARE_BASIC, BP_GLOBAL, BP_GLOBAL_BASIC, BP_TWOBIT
`define BPRED_SIZE 10
`define BTB_SIZE 10

`define SVADU_SUPPORTED 0
`define ZMMUL_SUPPORTED 0

// FPU division architecture
`define RADIX 32'h4
`define DIVCOPIES 32'h4

// bit manipulation
`define ZBA_SUPPORTED 0
`define ZBB_SUPPORTED 0
`define ZBC_SUPPORTED 0
`define ZBS_SUPPORTED 0

// Memory synthesis configuration
`define USE_SRAM 0




module alu #(parameter WIDTH=32) (
  input  logic [WIDTH-1:0] A, B,        // Operands
  input  logic             W64,         // W64-type instruction
  input  logic             SubArith,    // Subtraction or arithmetic shift
  input  logic [2:0]       ALUSelect,   // ALU mux select signal
  input  logic [1:0]       BSelect,     // Binary encoding of if it's a ZBA_ZBB_ZBC_ZBS instruction
  input  logic [2:0]       ZBBSelect,   // ZBB mux select signal
  input  logic [2:0]       Funct3,      // For BMU decoding
  input  logic [2:0]       BALUControl, // ALU Control signals for B instructions in Execute Stage
  output logic [WIDTH-1:0] ALUResult,   // ALU result
  output logic [WIDTH-1:0] Sum);        // Sum of operands

  // CondInvB = ~B when subtracting, B otherwise. Shift = shift result. SLT/U = result of a slt/u instruction.
  // FullResult = ALU result before adjusting for a RV64 w-suffix instruction.
  logic [WIDTH-1:0] CondMaskInvB, Shift, FullResult, PreALUResult;                // Intermediate Signals 
  logic [WIDTH-1:0] CondMaskB;                                                    // Result of B mask select mux
  logic [WIDTH-1:0] CondShiftA;                                                   // Result of A shifted select mux
  logic [WIDTH-1:0] CondExtA;                                                     // Result of Zero Extend A select mux
  logic             Carry, Neg;                                                   // Flags: carry out, negative
  logic             LT, LTU;                                                      // Less than, Less than unsigned
  logic             Asign, Bsign;                                                 // Sign bits of A, B

  // Addition
  assign CondMaskInvB = SubArith ? ~CondMaskB : CondMaskB;
  assign {Carry, Sum} = CondShiftA + CondMaskInvB + {{(WIDTH-1){1'b0}}, SubArith};
  
  // Shifts (configurable for rotation)
  shifter sh(.A, .Amt(B[`LOG_XLEN-1:0]), .Right(Funct3[2]), .W64, .SubArith, .Y(Shift), .Rotate(BALUControl[2]));

  // Condition code flags are based on subtraction output Sum = A-B.
  // Overflow occurs when the numbers being subtracted have the opposite sign 
  // and the result has the opposite sign of A.
  // LT is simplified from Overflow = Asign & Bsign & Asign & Neg; LT = Neg ^ Overflow
  assign Neg  = Sum[WIDTH-1];
  assign Asign = A[WIDTH-1];
  assign Bsign = B[WIDTH-1];
  assign LT = Asign & ~Bsign | Asign & Neg | ~Bsign & Neg; 
  assign LTU = ~Carry;
 
  // Select appropriate ALU Result
  always_comb begin
    case (ALUSelect)                                
      3'b000: FullResult = Sum;                           // add or sub (including address generation)
      3'b001: FullResult = Shift;                         // sll, sra, or srl
      3'b010: FullResult = {{(WIDTH-1){1'b0}}, LT};       // slt
      3'b011: FullResult = {{(WIDTH-1){1'b0}}, LTU};      // sltu
      3'b100: FullResult = A ^ CondMaskInvB;              // xor, xnor, binv
      3'b101: FullResult = (`ZBS_SUPPORTED | `ZBB_SUPPORTED) ? {{(WIDTH-1){1'b0}},{|(A & CondMaskB)}} : Shift; // bext (or IEU shift when BMU not supported)
      3'b110: FullResult = A | CondMaskInvB;              // or, orn, bset
      3'b111: FullResult = A & CondMaskInvB;              // and, bclr
    endcase
  end

  // Support RV64I W-type addw/subw/addiw/shifts that discard upper 32 bits and sign-extend 32-bit result to 64 bits
  if (WIDTH == 64)  assign PreALUResult = W64 ? {{32{FullResult[31]}}, FullResult[31:0]} : FullResult;
  else              assign PreALUResult = FullResult;

  // Final Result B instruction select mux
  if (`ZBC_SUPPORTED | `ZBS_SUPPORTED | `ZBA_SUPPORTED | `ZBB_SUPPORTED) begin : bitmanipalu
    bitmanipalu #(WIDTH) balu(.A, .B, .W64, .BSelect, .ZBBSelect, 
      .Funct3, .LT,.LTU, .BALUControl, .PreALUResult, .FullResult,
      .CondMaskB, .CondShiftA, .ALUResult);
  end else begin
    assign ALUResult = PreALUResult;
    assign CondMaskB = B;
    assign CondShiftA = A;
  end
endmodule



module shifter (
  input  logic [`XLEN-1:0]     A,                             // shift Source
  input  logic [`LOG_XLEN-1:0] Amt,                           // Shift amount
  input  logic                 Right, Rotate, W64, SubArith,  // Shift right, rotate, W64-type operation, arithmetic shift
  output logic [`XLEN-1:0]     Y);                            // Shifted result

  logic [2*`XLEN-2:0]          Z, ZShift;                     // Input to funnel shifter, shifted amount before truncated to 32 or 64 bits
  logic [`LOG_XLEN-1:0]        TruncAmt, Offset;              // Shift amount adjusted for RV64, right-shift amount
  logic                        Sign;                          // Sign bit for sign extension

  assign Sign = A[`XLEN-1] & SubArith;  // sign bit for sign extension
  if (`XLEN==32) begin // rv32
    if (`ZBB_SUPPORTED) begin: rotfunnel32 //rv32 shifter with rotates
      always_comb  // funnel mux
        case({Right, Rotate})
          2'b00: Z = {A[31:0], 31'b0};
          2'b01: Z = {A[31:0], A[31:1]};
          2'b10: Z = {{31{Sign}}, A[31:0]};
          2'b11: Z = {A[30:0], A[31:0]};
        endcase
    end else begin: norotfunnel32 //rv32 shifter without rotates
      always_comb  // funnel mux
        if (Right)  Z = {{31{Sign}}, A[31:0]};
        else        Z = {A[31:0], 31'b0};
    end
    assign TruncAmt = Amt; // shift amount
  end else begin // rv64
    logic [`XLEN-1:0]         A64;                            
    mux3 #(64) extendmux({{32{1'b0}}, A[31:0]}, {{32{A[31]}}, A[31:0]}, A, {~W64, SubArith}, A64); // bottom 32 bits are always A[31:0], so effectively a 32-bit upper mux
    if (`ZBB_SUPPORTED) begin: rotfunnel64 // rv64 shifter with rotates
      // shifter rotate source select mux
      logic [`XLEN-1:0]   RotA;                          // rotate source
      mux2 #(`XLEN) rotmux(A, {A[31:0], A[31:0]}, W64, RotA); // W64 rotatons
      always_comb  // funnel mux
        case ({Right, Rotate})
          2'b00: Z = {A64[63:0],{63'b0}};
          2'b01: Z = {RotA[63:0], RotA[63:1]};
          2'b10: Z = {{63{Sign}}, A64[63:0]};
          2'b11: Z = {RotA[62:0], RotA[63:0]};
        endcase
    end else begin: norotfunnel64 // rv64 shifter without rotates
      always_comb  // funnel mux
        if (Right)  Z = {{63{Sign}}, A64[63:0]};
        else        Z = {A64[63:0], {63'b0}};
    end
    assign TruncAmt = W64 ? {1'b0, Amt[4:0]} : Amt; // 32- or 64-bit shift
  end
  
  // Opposite offset for right shifts
  assign Offset = Right ? TruncAmt : ~TruncAmt;
  
  // Funnel operation
  assign ZShift = Z >> Offset;
  assign Y = ZShift[`XLEN-1:0];    
endmodule



///////////////////////////////////////////
// controller.sv
//
// Written: David_Harris@hmc.edu, Sarah.Harris@unlv.edu, kekim@hmc.edu
// Created: 9 January 2021
// Modified: 3 March 2023
//
// Purpose: Top level controller module
// 
// Documentation: RISC-V System on Chip Design Chapter 4 (Section 4.1.4, Figure 4.8, Table 4.5)
//
// A component of the CORE-V-WALLY configurable RISC-V project.
// 
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////



module controller(
  input  logic        clk, reset,
  // Decode stage control signals
  input  logic        StallD, FlushD,          // Stall, flush Decode stage
  input  logic [31:0] InstrD,                  // Instruction in Decode stage
  output logic [2:0]  ImmSrcD,                 // Type of immediate extension
  input  logic        IllegalIEUFPUInstrD,     // Illegal IEU and FPU instruction
  output logic        IllegalBaseInstrD,       // Illegal I-type instruction, or illegal RV32 access to upper 16 registers
  output logic        JumpD,                   // Jump instruction
  output logic        BranchD,                 // Branch instruction
   // Execute stage control signals             
  input  logic        StallE, FlushE,          // Stall, flush Execute stage
  input  logic [1:0]  FlagsE,                  // Comparison flags ({eq, lt})
  input  logic        FWriteIntE,              // Write integer register, coming from FPU controller
  output logic        PCSrcE,                  // Select signal to choose next PC (for datapath and Hazard unit)
  output logic        ALUSrcAE, ALUSrcBE,      // ALU operands
  output logic        ALUResultSrcE,           // Selects result to pass on to Memory stage
  output logic [2:0]  ALUSelectE,              // ALU mux select signal
  output logic        MemReadE, CSRReadE,      // Instruction reads memory, reads a CSR (needed for Hazard unit)
  output logic [2:0]  Funct3E,                 // Instruction's funct3 field
  output logic        IntDivE,                 // Integer divide
  output logic        MDUE,                    // MDU (multiply/divide) operatio
  output logic        W64E,                    // RV64 W-type operation
  output logic        SubArithE,               // Subtraction or arithmetic shift
  output logic        JumpE,                   // jump instruction
  output logic        BranchE,                 // Branch instruction
  output logic        SCE,                     // Store Conditional instruction
  output logic        BranchSignedE,           // Branch comparison operands are signed (if it's a branch)
  output logic [1:0]  BSelectE,                // One-Hot encoding of if it's ZBA_ZBB_ZBC_ZBS instruction
  output logic [2:0]  ZBBSelectE,              // ZBB mux select signal in Execute stage
  output logic [2:0]  BALUControlE,            // ALU Control signals for B instructions in Execute Stage

  // Memory stage control signals
  input  logic        StallM, FlushM,          // Stall, flush Memory stage
  output logic [1:0]  MemRWM,                  // Mem read/write: MemRWM[1] = 1 for read, MemRWM[0] = 1 for write 
  output logic        CSRReadM, CSRWriteM, PrivilegedM, // CSR read, write, or privileged instruction
  output logic [1:0]  AtomicM,                 // Atomic (AMO) instruction
  output logic [2:0]  Funct3M,                 // Instruction's funct3 field
  output logic        RegWriteM,               // Instruction writes a register (needed for Hazard unit)
  output logic        InvalidateICacheM, FlushDCacheM, // Invalidate I$, flush D$
  output logic        InstrValidD, InstrValidE, InstrValidM, // Instruction is valid
  output logic        FWriteIntM,              // FPU controller writes integer register file
  // Writeback stage control signals
  input  logic        StallW, FlushW,          // Stall, flush Writeback stage
  output logic        RegWriteW, IntDivW,      // Instruction writes a register, is an integer divide
  output logic [2:0]  ResultSrcW,              // Select source of result to write back to register file
  // Stall during CSRs
  output logic        CSRWriteFenceM,          // CSR write or fence instruction; needs to flush the following instructions
  output logic        StoreStallD              // Store (memory write) causes stall
);

  logic [6:0] OpD;                             // Opcode in Decode stage
  logic [2:0] Funct3D;                         // Funct3 field in Decode stage
  logic [6:0] Funct7D;                         // Funct7 field in Decode stage
  logic [4:0] Rs1D;                            // Rs1 source register in Decode stage

  `define CTRLW 23

  // pipelined control signals
  logic        RegWriteD, RegWriteE;           // RegWrite (register will be written)
  logic [2:0]  ResultSrcD, ResultSrcE, ResultSrcM; // Select which result to write back to register file
  logic [1:0]  MemRWD, MemRWE;                 // Store (write to memory)
  logic        ALUOpD;                         // 0 for address generation, 1 for all other operations (must use Funct3)
  logic        BaseW64D;                       // W64 for Base instructions specifically
  logic        BaseRegWriteD;                  // Indicates if Base instruction register write instruction
  logic        BaseSubArithD;                  // Indicates if Base instruction subtracts, sra, slt, sltu
  logic        BaseALUSrcBD;                   // Base instruction ALU B source select signal
  logic [2:0]  ALUControlD;                    // Determines ALU operation
  logic        ALUSrcAD, ALUSrcBD;             // ALU inputs
  logic        ALUResultSrcD, W64D, MDUD;      // ALU result, is RV64 W-type, is multiply/divide instruction
  logic        CSRZeroSrcD;                    // Ignore setting and clearing zeros to CSR
  logic        CSRReadD;                       // CSR read instruction
  logic [1:0]  AtomicD;                        // Atomic (AMO) instruction
  logic        FenceXD;                        // Fence instruction
  logic        InvalidateICacheD, FlushDCacheD;// Invalidate I$, flush D$
  logic        CSRWriteD, CSRWriteE;           // CSR write
  logic        PrivilegedD, PrivilegedE;       // Privileged instruction
  logic        InvalidateICacheE, FlushDCacheE;// Invalidate I$, flush D$
  logic [`CTRLW-1:0] ControlsD;                // Main Instruction Decoder control signals
  logic        SubArithD;                      // TRUE for R-type subtracts and sra, slt, sltu or B-type ext clr, andn, orn, xnor
  logic        subD, sraD, sltD, sltuD;        // Indicates if is one of these instructions
  logic        ALUOpE;                         // 0 for address generationm 1 for ALU operations
  logic        BranchTakenE;                   // Branch is taken
  logic        eqE, ltE;                       // Comparator outputs
  logic        unused; 
  logic        BranchFlagE;                    // Branch flag to use (chosen between eq or lt)
  logic        IEURegWriteE;                   // Register write 
  logic        BRegWriteE;                     // Register write from BMU controller in Execute Stage
  logic        IllegalERegAdrD;                // RV32E attempts to write upper 16 registers
  logic [1:0]  AtomicE;                        // Atomic instruction 
  logic        FenceD, FenceE;                 // Fence instruction
  logic        SFenceVmaD;                     // sfence.vma instruction
  logic        IntDivM;                        // Integer divide instruction
  logic [1:0]  BSelectD;                       // One-Hot encoding if it's ZBA_ZBB_ZBC_ZBS instruction in decode stage
  logic [2:0]  ZBBSelectD;                     // ZBB Mux Select Signal
  logic        IFunctD, RFunctD, MFunctD;      // Detect I, R, and M-type RV32IM/Rv64IM instructions
  logic        LFunctD, SFunctD, BFunctD;      // Detect load, store, branch instructions
  logic        JFunctD;                        // detect jalr instruction
  logic        FenceM;                         // Fence.I or sfence.VMA instruction in memory stage
  logic [2:0]  ALUSelectD;                     // ALU Output selection mux control
  logic        IWValidFunct3D;                 // Detects if Funct3 is valid for IW instructions

  // Extract fields
  assign OpD = InstrD[6:0];
  assign Funct3D = InstrD[14:12];
  assign Funct7D = InstrD[31:25];
  assign Rs1D = InstrD[19:15];

  // Funct 7 checking
  // Be rigorous about detecting illegal instructions if CSRs or bit manipulation is supported
  // otherwise be cheap

  if (`ZICSR_SUPPORTED | `ZBA_SUPPORTED | `ZBB_SUPPORTED | `ZBC_SUPPORTED | `ZBS_SUPPORTED) begin:legalcheck // Exact integer decoding
    logic Funct7ZeroD, Funct7b5D, IShiftD, INoShiftD;
    logic Funct7ShiftZeroD, Funct7Shiftb5D;

    assign Funct7ZeroD      = (Funct7D == 7'b0000000); // most R-type instructions
    assign Funct7b5D        = (Funct7D == 7'b0100000); // srai, sub
    assign Funct7ShiftZeroD = (`XLEN==64) ? (Funct7D[6:1] == 6'b000000) : Funct7ZeroD;
    assign Funct7Shiftb5D   = (`XLEN==64) ? (Funct7D[6:1] == 6'b010000) : Funct7b5D;
    assign IShiftD          = (Funct3D == 3'b001 & Funct7ShiftZeroD) | (Funct3D == 3'b101 & (Funct7ShiftZeroD | Funct7Shiftb5D)); // slli, srli, srai, or w forms
    assign INoShiftD        = ((Funct3D != 3'b001) & (Funct3D != 3'b101));
    assign IFunctD          = IShiftD | INoShiftD;
    assign RFunctD          = ((Funct3D == 3'b000 | Funct3D == 3'b101) & Funct7b5D) | Funct7ZeroD;
    assign MFunctD          = (Funct7D == 7'b0000001) & (`M_SUPPORTED | (`ZMMUL_SUPPORTED & ~Funct3D[2])); // muldiv
    assign LFunctD          = Funct3D == 3'b000 | Funct3D == 3'b001 | Funct3D == 3'b010 | Funct3D == 3'b100 | Funct3D == 3'b101 | 
                              ((`XLEN == 64) & (Funct3D == 3'b011 | Funct3D == 3'b110));
    assign SFunctD          = Funct3D == 3'b000 | Funct3D == 3'b001 | Funct3D == 3'b010 | 
                              ((`XLEN == 64) & (Funct3D == 3'b011));
    assign BFunctD          = (Funct3D[2:1] != 2'b01); // legal branches
    assign JFunctD          = (Funct3D == 3'b000);
    assign IWValidFunct3D   = Funct3D == 3'b000 | Funct3D == 3'b001 | Funct3D == 3'b101;
  end else begin:legalcheck2
    assign IFunctD = 1; // Don't bother to separate out shift decoding
    assign RFunctD = ~Funct7D[0]; // Not a multiply
    assign MFunctD = Funct7D[0] & (`M_SUPPORTED | (`ZMMUL_SUPPORTED & ~Funct3D[2])); // muldiv
    assign LFunctD = 1; // don't bother to check Funct3 for loads
    assign SFunctD = 1; // don't bother to check Funct3 for stores
    assign BFunctD = 1; // don't bother to check Funct3 for branches
    assign JFunctD = 1; // don't bother to check Funct3 for jumps
    assign IWValidFunct3D = 1;
  end

  // Main Instruction Decoder
  /* verilator lint_off CASEINCOMPLETE */
  always_comb begin
    ControlsD = `CTRLW'b0_000_00_00_000_0_0_0_0_0_0_0_0_0_00_1; // default: Illegal instruction
    case(OpD)
    // RegWrite_ImmSrc_ALUSrc_MemRW_ResultSrc_Branch_ALUOp_Jump_ALUResultSrc_W64_CSRRead_Privileged_Fence_MDU_Atomic_Illegal
     7'b0000011: if (LFunctD) 
                      ControlsD = `CTRLW'b1_000_01_10_001_0_0_0_0_0_0_0_0_0_00_0; // loads
      7'b0000111:     ControlsD = `CTRLW'b0_000_01_10_001_0_0_0_0_0_0_0_0_0_00_1; // flw - only legal if FP supported
      7'b0001111: if (`ZIFENCEI_SUPPORTED)
                      ControlsD = `CTRLW'b0_000_00_00_000_0_0_0_0_0_0_0_1_0_00_0; // fence
                  else
                      ControlsD = `CTRLW'b0_000_00_00_000_0_0_0_0_0_0_0_0_0_00_0; // fence treated as nop
      7'b0010011: if (IFunctD)    
                      ControlsD = `CTRLW'b1_000_01_00_000_0_1_0_0_0_0_0_0_0_00_0; // I-type ALU
      7'b0010111:     ControlsD = `CTRLW'b1_100_11_00_000_0_0_0_0_0_0_0_0_0_00_0; // auipc
      7'b0011011: if (IFunctD & IWValidFunct3D & `XLEN == 64)
                      ControlsD = `CTRLW'b1_000_01_00_000_0_1_0_0_1_0_0_0_0_00_0; // IW-type ALU for RV64i
      7'b0100011: if (SFunctD) 
                      ControlsD = `CTRLW'b0_001_01_01_000_0_0_0_0_0_0_0_0_0_00_0; // stores
      7'b0100111:     ControlsD = `CTRLW'b0_001_01_01_000_0_0_0_0_0_0_0_0_0_00_1; // fsw - only legal if FP supported
      7'b0101111: if (`A_SUPPORTED) begin
                    if (InstrD[31:27] == 5'b00010)
                      ControlsD = `CTRLW'b1_000_00_10_001_0_0_0_0_0_0_0_0_0_01_0; // lr
                    else if (InstrD[31:27] == 5'b00011)
                      ControlsD = `CTRLW'b1_101_01_01_100_0_0_0_0_0_0_0_0_0_01_0; // sc
                    else 
                      ControlsD = `CTRLW'b1_101_01_11_001_0_0_0_0_0_0_0_0_0_10_0; // amo
                 end
      7'b0110011: if (RFunctD)
                      ControlsD = `CTRLW'b1_000_00_00_000_0_1_0_0_0_0_0_0_0_00_0; // R-type 
                  else if (MFunctD)
                      ControlsD = `CTRLW'b1_000_00_00_011_0_0_0_0_0_0_0_0_1_00_0; // Multiply/divide
      7'b0110111:     ControlsD = `CTRLW'b1_100_01_00_000_0_0_0_1_0_0_0_0_0_00_0; // lui
      7'b0111011: if (RFunctD & (`XLEN == 64))
                      ControlsD = `CTRLW'b1_000_00_00_000_0_1_0_0_1_0_0_0_0_00_0; // R-type W instructions for RV64i
                  else if (MFunctD & (`XLEN == 64))
                      ControlsD = `CTRLW'b1_000_00_00_011_0_0_0_0_1_0_0_0_1_00_0; // W-type Multiply/Divide
      7'b1100011: if (BFunctD)   
                      ControlsD = `CTRLW'b0_010_11_00_000_1_0_0_0_0_0_0_0_0_00_0; // branches
      7'b1100111: if (JFunctD)
                      ControlsD = `CTRLW'b1_000_01_00_000_0_0_1_1_0_0_0_0_0_00_0; // jalr
      7'b1101111:     ControlsD = `CTRLW'b1_011_11_00_000_0_0_1_1_0_0_0_0_0_00_0; // jal
      7'b1110011: if (`ZICSR_SUPPORTED) begin
                   if (Funct3D == 3'b000)
                      ControlsD = `CTRLW'b0_000_00_00_000_0_0_0_0_0_0_1_0_0_00_0; // privileged; decoded further in priveleged modules
                   else
                      ControlsD = `CTRLW'b1_000_00_00_010_0_0_0_0_0_1_0_0_0_00_0; // csrs
                  end
    endcase
  end
  /* verilator lint_on CASEINCOMPLETE */

  // Unswizzle control bits
  // Squash control signals if coming from an illegal compressed instruction
  // On RV32E, can't write to upper 16 registers.  Checking reads to upper 16 is more costly so disregard them.
  assign IllegalERegAdrD = `E_SUPPORTED & `ZICSR_SUPPORTED & ControlsD[`CTRLW-1] & InstrD[11]; 
  //assign IllegalBaseInstrD = 1'b0;
  assign {BaseRegWriteD, ImmSrcD, ALUSrcAD, BaseALUSrcBD, MemRWD,
          ResultSrcD, BranchD, ALUOpD, JumpD, ALUResultSrcD, BaseW64D, CSRReadD, 
          PrivilegedD, FenceXD, MDUD, AtomicD, unused} = IllegalIEUFPUInstrD ? `CTRLW'b0 : ControlsD;
  
  assign CSRZeroSrcD = InstrD[14] ? (InstrD[19:15] == 0) : (Rs1D == 0); // Is a CSR instruction using zero as the source?
  assign CSRWriteD = CSRReadD & !(CSRZeroSrcD & InstrD[13]);            // Don't write if setting or clearing zeros
  assign SFenceVmaD = PrivilegedD & (InstrD[31:25] ==  7'b0001001);
  assign FenceD = SFenceVmaD | FenceXD; // possible sfence.vma or fence.i
  
  // ALU Decoding is lazy, only using func7[5] to distinguish add/sub and srl/sra
  assign sltuD = (Funct3D == 3'b011); 
  assign subD = (Funct3D == 3'b000 & Funct7D[5] & OpD[5]);  // OpD[5] needed to distinguish sub from addi
  assign sraD = (Funct3D == 3'b101 & Funct7D[5]);
  assign BaseSubArithD = ALUOpD & (subD | sraD | sltD | sltuD);

  // bit manipulation Configuration Block
  if (`ZBS_SUPPORTED | `ZBA_SUPPORTED | `ZBB_SUPPORTED | `ZBC_SUPPORTED) begin: bitmanipi //change the conditional expression to OR any Z supported flags
    logic IllegalBitmanipInstrD;          // Unrecognized B instruction
    logic BRegWriteD;                     // Indicates if it is a R type BMU instruction in decode stage
    logic BW64D;                          // Indicates if it is a W type BMU instruction in decode stage
    logic BSubArithD;                     // TRUE for BMU ext, clr, andn, orn, xnor
    logic BALUSrcBD;                      // BMU alu src select signal

    bmuctrl bmuctrl(.clk, .reset, .StallD, .FlushD, .InstrD, .ALUOpD, .BSelectD, .ZBBSelectD, 
      .BRegWriteD, .BALUSrcBD, .BW64D, .BSubArithD, .IllegalBitmanipInstrD, .StallE, .FlushE, 
      .ALUSelectD, .BSelectE, .ZBBSelectE, .BRegWriteE, .BALUControlE);
    if (`ZBA_SUPPORTED) begin
      // ALU Decoding is more comprehensive when ZBA is supported. slt and slti conflicts with sh1add, sh1add.uw
      assign sltD = (Funct3D == 3'b010 & (~(Funct7D[4]) | ~OpD[5])) ;
    end else assign sltD = (Funct3D == 3'b010);

    // Combine base and bit manipulation signals
    // coverage off: IllegalERegAdr can't occur in rv64gc; only applicable to E mode
    assign IllegalBaseInstrD = (ControlsD[0] & IllegalBitmanipInstrD) | IllegalERegAdrD ;
    // coverage on
    assign RegWriteD = BaseRegWriteD | BRegWriteD; 
    assign W64D = BaseW64D | BW64D;
    assign ALUSrcBD = BaseALUSrcBD | BALUSrcBD;
    assign SubArithD = BaseSubArithD | BSubArithD; // TRUE If BMU or R-type instruction involves inverted operand

  end else begin: bitmanipi
    assign ALUSelectD = ALUOpD ? Funct3D : 3'b000; // add for address generation when not doing ALU operation
    assign sltD = (Funct3D == 3'b010);
    assign IllegalBaseInstrD = ControlsD[0] | IllegalERegAdrD ;
    assign RegWriteD = BaseRegWriteD; 
    assign W64D = BaseW64D;
    assign ALUSrcBD = BaseALUSrcBD;
    assign SubArithD = BaseSubArithD; // TRUE If B-type or R-type instruction involves inverted operand

    // tie off unused bit manipulation signals
    assign BSelectE = 2'b00;
    assign BSelectD = 2'b00;
    assign ZBBSelectE = 3'b000;
    assign BALUControlE = 3'b0;
  end

  // Fences
  // Ordinary fence is presently a nop
  // fence.i flushes the D$ and invalidates the I$ if Zifencei is supported and I$ is implemented
  if (`ZIFENCEI_SUPPORTED & `ICACHE_SUPPORTED) begin:fencei
    logic FenceID;
    assign FenceID = FenceXD & (Funct3D == 3'b001); // is it a FENCE.I instruction?
    assign InvalidateICacheD = FenceID;
    assign FlushDCacheD = FenceID;
  end else begin:fencei
    assign InvalidateICacheD = 0;
    assign FlushDCacheD = 0;
  end
 
  // Decode stage pipeline control register
  flopenrc #(1)  controlregD(clk, reset, FlushD, ~StallD, 1'b1, InstrValidD);

  // Execute stage pipeline control register and logic
  flopenrc #(29) controlregE(clk, reset, FlushE, ~StallE,
                           {ALUSelectD, RegWriteD, ResultSrcD, MemRWD, JumpD, BranchD, ALUSrcAD, ALUSrcBD, ALUResultSrcD, CSRReadD, CSRWriteD, PrivilegedD, Funct3D, W64D, SubArithD, MDUD, AtomicD, InvalidateICacheD, FlushDCacheD, FenceD, InstrValidD},
                           {ALUSelectE, IEURegWriteE, ResultSrcE, MemRWE, JumpE, BranchE, ALUSrcAE, ALUSrcBE, ALUResultSrcE, CSRReadE, CSRWriteE, PrivilegedE, Funct3E, W64E, SubArithE, MDUE, AtomicE, InvalidateICacheE, FlushDCacheE, FenceE, InstrValidE});

  // Branch Logic
  //  The comparator handles both signed and unsigned branches using BranchSignedE
  //  Hence, only eq and lt flags are needed
  assign BranchSignedE = (~(Funct3E[2:1] == 2'b11) & BranchE);
  assign {eqE, ltE} = FlagsE;
  mux2 #(1) branchflagmux(eqE, ltE, Funct3E[2], BranchFlagE);
  assign BranchTakenE = BranchFlagE ^ Funct3E[0];
  assign PCSrcE = JumpE | BranchE & BranchTakenE;

  // Other execute stage controller signals
  assign MemReadE = MemRWE[1];
  assign SCE = (ResultSrcE == 3'b100);
  assign RegWriteE = IEURegWriteE | FWriteIntE; // IRF register writes could come from IEU or FPU controllers
  assign IntDivE = MDUE & Funct3E[2]; // Integer division operation
  
  // Memory stage pipeline control register
  flopenrc #(20) controlregM(clk, reset, FlushM, ~StallM,
                         {RegWriteE, ResultSrcE, MemRWE, CSRReadE, CSRWriteE, PrivilegedE, Funct3E, FWriteIntE, AtomicE, InvalidateICacheE, FlushDCacheE, FenceE, InstrValidE, IntDivE},
                         {RegWriteM, ResultSrcM, MemRWM, CSRReadM, CSRWriteM, PrivilegedM, Funct3M, FWriteIntM, AtomicM, InvalidateICacheM, FlushDCacheM, FenceM, InstrValidM, IntDivM});
  
  // Writeback stage pipeline control register
  flopenrc #(5) controlregW(clk, reset, FlushW, ~StallW,
                         {RegWriteM, ResultSrcM, IntDivM},
                         {RegWriteW, ResultSrcW, IntDivW});  

  // Flush F, D, and E stages on a CSR write or Fence.I or SFence.VMA
  assign CSRWriteFenceM = CSRWriteM | FenceM;

  // the synchronous DTIM cannot read immediately after write
  // a cache cannot read or write immediately after a write
  assign StoreStallD = MemRWE[0] & ((MemRWD[1] | (MemRWD[0] & `DCACHE_SUPPORTED)) | (|AtomicD));
endmodule



module extend (
  input  logic [31:7]       InstrD,      // All instruction bits except opcode (lower 7 bits)
  input  logic [2:0]        ImmSrcD,     // Select what kind of extension to perform
  output logic [`XLEN-1:0 ] ImmExtD);    // Extended immediate

  localparam [`XLEN-1:0] undefined = {(`XLEN){1'bx}}; // could change to 0 after debug
 
  always_comb
    case(ImmSrcD) 
      // I-type 
      3'b000:   ImmExtD = {{(`XLEN-12){InstrD[31]}}, InstrD[31:20]};  
      // S-type (stores)
      3'b001:   ImmExtD = {{(`XLEN-12){InstrD[31]}}, InstrD[31:25], InstrD[11:7]}; 
      // B-type (branches)
      3'b010:   ImmExtD = {{(`XLEN-12){InstrD[31]}}, InstrD[7], InstrD[30:25], InstrD[11:8], 1'b0}; 
      // J-type (jal)
      3'b011:   ImmExtD = {{(`XLEN-20){InstrD[31]}}, InstrD[19:12], InstrD[20], InstrD[30:21], 1'b0}; 
      // U-type (lui, auipc)
      3'b100:  ImmExtD = {{(`XLEN-31){InstrD[31]}}, InstrD[30:12], 12'b0}; 
      // Store Conditional: zero offset
      3'b101:  if (`A_SUPPORTED) ImmExtD = 0;
               else              ImmExtD = undefined;
      default: ImmExtD = undefined; // undefined
    endcase  
endmodule



// This comparator is best
module comparator #(parameter WIDTH=64) (
  input  logic [WIDTH-1:0] a, b,    // Operands
  input  logic             sgnd,    // Signed operands
  output logic [1:0]       flags);  // Output flags: {eq, lt}

  logic             eq, lt;         // Flags: equal (eq), less than (lt)
  logic [WIDTH-1:0] af, bf;         // Operands with msb flipped (inverted) when signed

  // For signed numbers, flip most significant bit
  assign af = {a[WIDTH-1] ^ sgnd, a[WIDTH-2:0]};
  assign bf = {b[WIDTH-1] ^ sgnd, b[WIDTH-2:0]};

  // Behavioral description gives best results
  assign eq = (a == b);            // eq = 1 when operands are equal, 0 otherwise
  assign lt = (af < bf);           // lt = 1 when a less than b (taking signed operands into account)
  assign flags = {eq, lt};
endmodule


///////////////////////////////////////////
// datapath.sv
//
// Written: David_Harris@hmc.edu, Sarah.Harris@unlv.edu
// Created: 9 January 2021
// Modified: 
//
// Purpose: Wally Integer Datapath
// 
// Documentation: RISC-V System on Chip Design Chapter 4 (Figure 4.12)
//
// A component of the CORE-V-WALLY configurable RISC-V project.
// 
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////


module datapath (
  input  logic             clk, reset,
  // Decode stage signals
  input  logic [2:0]       ImmSrcD,                 // Selects type of immediate extension
  input  logic [31:0]      InstrD,                  // Instruction in Decode stage
  // Execute stage signals
  input  logic [`XLEN-1:0] PCE,                     // PC in Execute stage  
  input  logic [`XLEN-1:0] PCLinkE,                 // PC + 4 (of instruction in Execute stage)
  input  logic [2:0]       Funct3E,                 // Funct3 field of instruction in Execute stage
  input  logic             StallE, FlushE,          // Stall, flush Execute stage
  input  logic [1:0]       ForwardAE, ForwardBE,    // Forward ALU operands from later stages
  input  logic             W64E,                    // W64-type instruction
  input  logic             SubArithE,               // Subtraction or arithmetic shift
  input  logic             ALUSrcAE, ALUSrcBE,      // ALU operands
  input  logic             ALUResultSrcE,           // Selects result to pass on to Memory stage
  input  logic [2:0]       ALUSelectE,              // ALU mux select signal
  input  logic             JumpE,                   // Is a jump (j) instruction
  input  logic             BranchSignedE,           // Branch comparison operands are signed (if it's a branch)
  input  logic [1:0]       BSelectE,                // One hot encoding of ZBA_ZBB_ZBC_ZBS instruction
  input  logic [2:0]       ZBBSelectE,              // ZBB mux select signal
  input  logic [2:0]       BALUControlE,            // ALU Control signals for B instructions in Execute Stage
  output logic [1:0]       FlagsE,                  // Comparison flags ({eq, lt})
  output logic [`XLEN-1:0] IEUAdrE,                 // Address computed by ALU
  output logic [`XLEN-1:0] ForwardedSrcAE, ForwardedSrcBE, // ALU sources before the mux chooses between them and PCE to put in srcA/B
  // Memory stage signals
  input  logic             StallM, FlushM,          // Stall, flush Memory stage
  input  logic             FWriteIntM, FCvtIntW,    // FPU writes integer register file, FPU converts float to int
  input  logic [`XLEN-1:0] FIntResM,                // FPU integer result
  output logic [`XLEN-1:0] SrcAM,                   // ALU's Source A in Memory stage to privilege unit for CSR writes
  output logic [`XLEN-1:0] WriteDataM,              // Write data in Memory stage
  // Writeback stage signals
  input  logic             StallW, FlushW,          // Stall, flush Writeback stage
  input  logic             RegWriteW, IntDivW,      // Write register file, integer divide instruction
  input  logic             SquashSCW,               // Squash a store conditional when a conflict arose
  input  logic [2:0]       ResultSrcW,              // Select source of result to write back to register file
  input  logic [`XLEN-1:0] FCvtIntResW,             // FPU convert fp to integer result
  input  logic [`XLEN-1:0] ReadDataW,               // Read data from LSU
  input  logic [`XLEN-1:0] CSRReadValW,             // CSR read result
  input  logic [`XLEN-1:0] MDUResultW,              // MDU (Multiply/divide unit) result
  input  logic [`XLEN-1:0] FIntDivResultW,          // FPU's integer divide result
   // Hazard Unit signals 
  output logic [4:0]       Rs1D, Rs2D, Rs1E, Rs2E,  // Register sources to read in Decode or Execute stage
  output logic [4:0]       RdE, RdM, RdW            // Register destinations in Execute, Memory, or Writeback stage
);

  // Fetch stage signals
  // Decode stage signals
  logic [`XLEN-1:0] R1D, R2D;                       // Read data from Rs1 (RD1), Rs2 (RD2)
  logic [`XLEN-1:0] ImmExtD;                        // Extended immediate in Decode stage
  logic [4:0]       RdD;                            // Destination register in Decode stage
  // Execute stage signals
  logic [`XLEN-1:0] R1E, R2E;                       // Source operands read from register file
  logic [`XLEN-1:0] ImmExtE;                        // Extended immediate in Execute stage 
  logic [`XLEN-1:0] SrcAE, SrcBE;                   // ALU operands
  logic [`XLEN-1:0] ALUResultE, AltResultE, IEUResultE; // ALU result, Alternative result (ImmExtE or PC+4), result of execution stage
  // Memory stage signals
  logic [`XLEN-1:0] IEUResultM;                     // Result from execution stage
  logic [`XLEN-1:0] IFResultM;                      // Result from either IEU or single-cycle FPU op writing an integer register
  // Writeback stage signals
  logic [`XLEN-1:0] SCResultW;                      // Store Conditional result
  logic [`XLEN-1:0] ResultW;                        // Result to write to register file
  logic [`XLEN-1:0] IFResultW;                      // Result from either IEU or single-cycle FPU op writing an integer register
  logic [`XLEN-1:0] IFCvtResultW;                   // Result from IEU, signle-cycle FPU op, or 2-cycle FCVT float to int 
  logic [`XLEN-1:0] MulDivResultW;                  // Multiply always comes from MDU.  Divide could come from MDU or FPU (when using fdivsqrt for integer division)

  // Decode stage
  assign Rs1D      = InstrD[19:15];
  assign Rs2D      = InstrD[24:20];
  assign RdD       = InstrD[11:7];
  regfile regf(clk, reset, RegWriteW, Rs1D, Rs2D, RdW, ResultW, R1D, R2D);
  extend ext(.InstrD(InstrD[31:7]), .ImmSrcD, .ImmExtD);
 
  // Execute stage pipeline register and logic
  flopenrc #(`XLEN) RD1EReg(clk, reset, FlushE, ~StallE, R1D, R1E);
  flopenrc #(`XLEN) RD2EReg(clk, reset, FlushE, ~StallE, R2D, R2E);
  flopenrc #(`XLEN) ImmExtEReg(clk, reset, FlushE, ~StallE, ImmExtD, ImmExtE);
  flopenrc #(5)     Rs1EReg(clk, reset, FlushE, ~StallE, Rs1D, Rs1E);
  flopenrc #(5)     Rs2EReg(clk, reset, FlushE, ~StallE, Rs2D, Rs2E);
  flopenrc #(5)     RdEReg(clk, reset, FlushE, ~StallE, RdD, RdE);
  
  mux3  #(`XLEN)  faemux(R1E, ResultW, IFResultM, ForwardAE, ForwardedSrcAE);
  mux3  #(`XLEN)  fbemux(R2E, ResultW, IFResultM, ForwardBE, ForwardedSrcBE);
  comparator #(`XLEN) comp(ForwardedSrcAE, ForwardedSrcBE, BranchSignedE, FlagsE);
  mux2  #(`XLEN)  srcamux(ForwardedSrcAE, PCE, ALUSrcAE, SrcAE);
  mux2  #(`XLEN)  srcbmux(ForwardedSrcBE, ImmExtE, ALUSrcBE, SrcBE);
  alu   #(`XLEN)  alu(SrcAE, SrcBE, W64E, SubArithE, ALUSelectE, BSelectE, ZBBSelectE, Funct3E, BALUControlE, ALUResultE, IEUAdrE);
  mux2 #(`XLEN)   altresultmux(ImmExtE, PCLinkE, JumpE, AltResultE);
  mux2 #(`XLEN)   ieuresultmux(ALUResultE, AltResultE, ALUResultSrcE, IEUResultE);

  // Memory stage pipeline register
  flopenrc #(`XLEN) SrcAMReg(clk, reset, FlushM, ~StallM, SrcAE, SrcAM);
  flopenrc #(`XLEN) IEUResultMReg(clk, reset, FlushM, ~StallM, IEUResultE, IEUResultM);
  flopenrc #(5)     RdMReg(clk, reset, FlushM, ~StallM, RdE, RdM);  
  flopenrc #(`XLEN) WriteDataMReg(clk, reset, FlushM, ~StallM, ForwardedSrcBE, WriteDataM); 
  
  // Writeback stage pipeline register and logic
  flopenrc #(`XLEN) IFResultWReg(clk, reset, FlushW, ~StallW, IFResultM, IFResultW);
  flopenrc #(5)     RdWReg(clk, reset, FlushW, ~StallW, RdM, RdW);

  // floating point inputs: FIntResM comes from fclass, fcmp, fmv; FCvtIntResW comes from fcvt
  if (`F_SUPPORTED) begin:fpmux
    mux2  #(`XLEN)  resultmuxM(IEUResultM, FIntResM, FWriteIntM, IFResultM);
    mux2  #(`XLEN)  cvtresultmuxW(IFResultW, FCvtIntResW, FCvtIntW, IFCvtResultW);
    if (`IDIV_ON_FPU) begin
      mux2  #(`XLEN)  divresultmuxW(MDUResultW, FIntDivResultW, IntDivW, MulDivResultW);
    end else begin 
      assign MulDivResultW = MDUResultW;
    end
  end else begin:fpmux
    assign IFResultM = IEUResultM; 
    assign IFCvtResultW = IFResultW;
    assign MulDivResultW = MDUResultW;
  end
  mux5  #(`XLEN)  resultmuxW(IFCvtResultW, ReadDataW, CSRReadValW, MulDivResultW, SCResultW, ResultSrcW, ResultW); 
 
  // handle Store Conditional result if atomic extension supported
  if (`A_SUPPORTED) assign SCResultW = {{(`XLEN-1){1'b0}}, SquashSCW};
  else              assign SCResultW = 0;
endmodule



module forward(
  // Detect hazards
  input  logic [4:0]  Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW, // Source and destination registers
  input  logic        MemReadE, MDUE, CSRReadE,              // Execute stage instruction is a load (MemReadE), divide (MDUE), or CSR read (CSRReadE)
  input  logic        RegWriteM, RegWriteW,                  // Instruction in Memory or Writeback stage writes register file
  input  logic        FCvtIntE,                              // FPU convert float to int
  input  logic        SCE,                                   // Store Conditional instruction
  // Forwarding controls
  output logic [1:0]  ForwardAE, ForwardBE,                  // Select signals for forwarding multiplexers
  output logic        FCvtIntStallD, LoadStallD, MDUStallD, CSRRdStallD // Stall due to conversion, load, multiply/divide, CSR read
);

  logic MatchDE;                                             // Match between a source register in Decode stage and destination register in Execute stage
  
  always_comb begin
    ForwardAE = 2'b00;
    ForwardBE = 2'b00;
    if (Rs1E != 5'b0)
      if      ((Rs1E == RdM) & RegWriteM) ForwardAE = 2'b10;
      else if ((Rs1E == RdW) & RegWriteW) ForwardAE = 2'b01;
 
    if (Rs2E != 5'b0)
      if      ((Rs2E == RdM) & RegWriteM) ForwardBE = 2'b10;
      else if ((Rs2E == RdW) & RegWriteW) ForwardBE = 2'b01;
  end

  // Stall on dependent operations that finish in Mem Stage and can't bypass in time
  assign MatchDE = ((Rs1D == RdE) | (Rs2D == RdE)) & (RdE != 5'b0); // Decode-stage instruction source depends on result from execute stage instruction
  assign FCvtIntStallD = FCvtIntE & MatchDE; // FPU to Integer transfers have single-cycle latency except fcvt
  assign LoadStallD = (MemReadE|SCE) & MatchDE;  
  assign MDUStallD = MDUE & MatchDE; // Int mult/div is at least two cycle latency, even when coming from the FDIV
  assign CSRRdStallD = CSRReadE & MatchDE;
endmodule



module regfile (
  input  logic             clk, reset,
  input  logic             we3,                 // Write enable
  input  logic [4:0]       a1, a2, a3,          // Source registers to read (a1, a2), destination register to write (a3)
  input  logic [`XLEN-1:0] wd3,                 // Write data for port 3
  output logic [`XLEN-1:0] rd1, rd2);           // Read data for ports 1, 2

  localparam NUMREGS = `E_SUPPORTED ? 16 : 32;  // only 16 registers in E mode

  logic [`XLEN-1:0] rf[NUMREGS-1:1];
  integer i;

  // Three ported register file
  // Read two ports combinationally (a1/rd1, a2/rd2)
  // Write third port on rising edge of clock (a3/wd3/we3)
  // Write occurs on falling edge of clock
  // Register 0 hardwired to 0
  
  // reset is intended for simulation only, not synthesis
  // can logic be adjusted to not need resettable registers?
    
  always_ff @(negedge clk)
    if (reset) for(i=1; i<NUMREGS; i++) rf[i] <= 0;
    else       if (we3)            rf[a3] <= wd3;  

  assign #2 rd1 = (a1 != 0) ? rf[a1] : 0;
  assign #2 rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule



module ieu (
  input  logic              clk, reset,
  // Decode stage signals
  input  logic [31:0]       InstrD,                          // Instruction
  input  logic              IllegalIEUFPUInstrD,             // Illegal instruction
  output logic              IllegalBaseInstrD,               // Illegal I-type instruction, or illegal RV32 access to upper 16 registers
  // Execute stage signals
  input  logic [`XLEN-1:0]  PCE,                             // PC
  input  logic [`XLEN-1:0]  PCLinkE,                         // PC + 4
  output logic              PCSrcE,                          // Select next PC (between PC+4 and IEUAdrE)
  input  logic              FWriteIntE, FCvtIntE,            // FPU writes to integer register file, FPU converts float to int
  output logic [`XLEN-1:0]  IEUAdrE,                         // Memory address
  output logic              IntDivE, W64E,                   // Integer divide, RV64 W-type instruction 
  output logic [2:0]        Funct3E,                         // Funct3 instruction field
  output logic [`XLEN-1:0]  ForwardedSrcAE, ForwardedSrcBE,  // ALU src inputs before the mux choosing between them and PCE to put in srcA/B
  output logic [4:0]        RdE,                             // Destination register
  // Memory stage signals
  input  logic              SquashSCW,                       // Squash store conditional, from LSU
  output logic [1:0]        MemRWM,                          // Read/write control goes to LSU
  output logic [1:0]        AtomicM,                         // Atomic control goes to LSU
  output logic [`XLEN-1:0]  WriteDataM,                      // Write data to LSU
  output logic [2:0]        Funct3M,                         // Funct3 (size and signedness) to LSU
  output logic [`XLEN-1:0]  SrcAM,                           // ALU SrcA to Privileged unit and FPU
  output logic [4:0]        RdM,                             // Destination register
  input  logic [`XLEN-1:0]  FIntResM,                        // Integer result from FPU (fmv, fclass, fcmp)
  output logic              InvalidateICacheM, FlushDCacheM, // Invalidate I$, flush D$
  output logic              InstrValidD, InstrValidE, InstrValidM,// Instruction is valid
  output logic              BranchD, BranchE,
  output logic              JumpD, JumpE,
  // Writeback stage signals
  input  logic [`XLEN-1:0]  FIntDivResultW,                  // Integer divide result from FPU fdivsqrt)
  input  logic [`XLEN-1:0]  CSRReadValW,                     // CSR read value, 
  input  logic [`XLEN-1:0]  MDUResultW,                      // multiply/divide unit result
  input  logic [`XLEN-1:0]  FCvtIntResW,                     // FPU's float to int conversion result
  input  logic              FCvtIntW,                        // FPU converts float to int
  output logic [4:0]        RdW,                             // Destination register
  input  logic [`XLEN-1:0]  ReadDataW,                       // LSU's read data
  // Hazard unit signals
  input  logic              StallD, StallE, StallM, StallW,  // Stall signals from hazard unit
  input  logic              FlushD, FlushE, FlushM, FlushW,  // Flush signals
  output logic              FCvtIntStallD, LoadStallD,       // Stall causes from IEU to hazard unit
  output logic              MDUStallD, CSRRdStallD, StoreStallD,
  output logic              CSRReadM, CSRWriteM, PrivilegedM,// CSR read, CSR write, is privileged instruction
  output logic              CSRWriteFenceM                   // CSR write or fence instruction needs to flush subsequent instructions
);

  logic [2:0] ImmSrcD;                                       // Select type of immediate extension 
  logic [1:0] FlagsE;                                        // Comparison flags ({eq, lt})
  logic       ALUSrcAE, ALUSrcBE;                            // ALU source operands
  logic [2:0] ResultSrcW;                                    // Selects result in Writeback stage
  logic       ALUResultSrcE;                                 // Selects ALU result to pass on to Memory stage
  logic [2:0] ALUSelectE;                                    // ALU select mux signal
  logic       SCE;                                           // Store Conditional instruction
  logic       FWriteIntM;                                    // FPU writing to integer register file
  logic       IntDivW;                                       // Integer divide instruction
  logic [1:0] BSelectE;                                      // Indicates if ZBA_ZBB_ZBC_ZBS instruction in one-hot encoding
  logic [2:0] ZBBSelectE;                                    // ZBB Result Select Signal in Execute Stage
  logic [2:0] BALUControlE;                                  // ALU Control signals for B instructions in Execute Stage
  logic       SubArithE;                                     // Subtraction or arithmetic shift

  // Forwarding signals
  logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E;                        // Source and destination registers
  logic [1:0] ForwardAE, ForwardBE;                          // Select signals for forwarding multiplexers
  logic       RegWriteM, RegWriteW;                          // Register will be written in Memory, Writeback stages
  logic       MemReadE, CSRReadE;                            // Load, CSRRead instruction
  logic       BranchSignedE;                                 // Branch does signed comparison on operands
  logic       MDUE;                                          // Multiply/divide instruction
           
controller c(
    .clk, .reset, .StallD, .FlushD, .InstrD, .ImmSrcD,
    .IllegalIEUFPUInstrD, .IllegalBaseInstrD, .StallE, .FlushE, .FlagsE, .FWriteIntE,
    .PCSrcE, .ALUSrcAE, .ALUSrcBE, .ALUResultSrcE, .ALUSelectE, .MemReadE, .CSRReadE, 
    .Funct3E, .IntDivE, .MDUE, .W64E, .SubArithE, .BranchD, .BranchE, .JumpD, .JumpE, .SCE, .BranchSignedE, .BSelectE, .ZBBSelectE, .BALUControlE, .StallM, .FlushM, .MemRWM,
    .CSRReadM, .CSRWriteM, .PrivilegedM, .AtomicM, .Funct3M,
    .RegWriteM, .FlushDCacheM, .InstrValidM, .InstrValidE, .InstrValidD, .FWriteIntM,
    .StallW, .FlushW, .RegWriteW, .IntDivW, .ResultSrcW, .CSRWriteFenceM, .InvalidateICacheM, .StoreStallD);

  datapath   dp(
    .clk, .reset, .ImmSrcD, .InstrD, .StallE, .FlushE, .ForwardAE, .ForwardBE, .W64E, .SubArithE,
    .Funct3E, .ALUSrcAE, .ALUSrcBE, .ALUResultSrcE, .ALUSelectE, .JumpE, .BranchSignedE, 
    .PCE, .PCLinkE, .FlagsE, .IEUAdrE, .ForwardedSrcAE, .ForwardedSrcBE, .BSelectE, .ZBBSelectE, .BALUControlE,
    .StallM, .FlushM, .FWriteIntM, .FIntResM, .SrcAM, .WriteDataM, .FCvtIntW,
    .StallW, .FlushW, .RegWriteW, .IntDivW, .SquashSCW, .ResultSrcW, .ReadDataW, .FCvtIntResW,
    .CSRReadValW, .MDUResultW, .FIntDivResultW, .Rs1D, .Rs2D, .Rs1E, .Rs2E, .RdE, .RdM, .RdW);             
  
  forward    fw(
    .Rs1D, .Rs2D, .Rs1E, .Rs2E, .RdE, .RdM, .RdW,
    .MemReadE, .MDUE, .CSRReadE, .RegWriteM, .RegWriteW,
    .FCvtIntE, .SCE, .ForwardAE, .ForwardBE,
    .FCvtIntStallD, .LoadStallD, .MDUStallD, .CSRRdStallD);
endmodule






module testbench();

   logic        clk;
   logic        reset;

   logic [31:0] WriteData, DataAdr;
   logic        MemWrite;

   // instantiate device to be tested
   top dut(clk, reset, WriteData, DataAdr, MemWrite);

   initial
     begin
	string memfilename;
        memfilename = {"../riscvtest/riscvtest.memfile"};
	$readmemh(memfilename, dut.imem.RAM);
     end
   
   // initialize test
   initial
     begin
	reset <= 1; # 22; reset <= 0;
     end

   // generate clock to sequence tests
   always
     begin
	clk <= 1; # 5; clk <= 0; # 5;
     end

   // check results
   always @(negedge clk)
     begin
	if(MemWrite) begin
           if(DataAdr === 100 & WriteData === 25) begin
              $display("Simulation succeeded");
              $stop;
           end else if (DataAdr !== 96) begin
              $display("Simulation failed");
              $stop;
           end
	end
     end
endmodule

module top(input  logic        clk, reset, 
           output logic [31:0] WriteDataM, DataAdrM, 
           output logic        MemWriteM);

   logic [31:0] 	       PCF, InstrF, ReadDataM;
   
   // instantiate processor and memories
   riscv rv32pipe (clk, reset, PCF, InstrF, MemWriteM, DataAdrM, 
		   WriteDataM, ReadDataM);
   imem imem (PCF, InstrF);
   dmem dmem (clk, MemWriteM, DataAdrM, WriteDataM, ReadDataM);
   
endmodule

module riscv(input  logic        clk, reset,
             output logic [31:0] PCF,
             input logic [31:0]  InstrF,
             output logic 	 MemWriteM,
             output logic [31:0] ALUResultM, WriteDataM,
             input logic [31:0]  ReadDataM);

   logic [6:0] 			 opD;
   logic [2:0] 			 funct3D;
   logic 			 funct7b5D;
   logic [1:0] 			 ImmSrcD;
   logic 			 ZeroE;
   logic 			 PCSrcE;
   logic [2:0] 			 ALUControlE;
   logic 			 ALUSrcE;
   logic 			 ResultSrcEb0;
   logic 			 RegWriteM;
   logic [1:0] 			 ResultSrcW;
   logic 			 RegWriteW;

   logic [1:0] 			 ForwardAE, ForwardBE;
   logic 			 StallF, StallD, FlushD, FlushE;

   logic [4:0] 			 Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW;
   
   controller c(clk, reset,
		opD, funct3D, funct7b5D, ImmSrcD,
		FlushE, ZeroE, PCSrcE, ALUControlE, ALUSrcE, ResultSrcEb0,
		MemWriteM, RegWriteM, 
		RegWriteW, ResultSrcW);

   datapath dp(clk, reset,
               StallF, PCF, InstrF,
	       opD, funct3D, funct7b5D, StallD, FlushD, ImmSrcD,
	       FlushE, ForwardAE, ForwardBE, PCSrcE, ALUControlE, ALUSrcE, ZeroE,
               MemWriteM, WriteDataM, ALUResultM, ReadDataM,
               RegWriteW, ResultSrcW,
               Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW);

   hazard  hu(Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
              PCSrcE, ResultSrcEb0, RegWriteM, RegWriteW,
              ForwardAE, ForwardBE, StallF, StallD, FlushD, FlushE);			 
endmodule



module flopr #(parameter WIDTH = 8)
   (input  logic             clk, reset,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset) q <= 0;
     else       q <= d;
endmodule

module flopenr #(parameter WIDTH = 8)
   (input  logic             clk, reset, en,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset)   q <= 0;
     else if (en) q <= d;
endmodule

module flopenrc #(parameter WIDTH = 8)
   (input  logic             clk, reset, clear, en,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset)   q <= 0;
     else if (en) 
       if (clear) q <= 0;
       else       q <= d;
endmodule

module floprc #(parameter WIDTH = 8)
   (input  logic clk,
    input logic 	     reset,
    input logic 	     clear,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset) q <= 0;
     else       
       if (clear) q <= 0;
       else       q <= d;
endmodule


module mux2 #(parameter WIDTH = 8) (
  input  logic [WIDTH-1:0] d0, d1, 
  input  logic             s, 
  output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8) (
  input  logic [WIDTH-1:0] d0, d1, d2,
  input  logic [1:0]       s, 
  output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); // exclusion-tag: mux3
endmodule

module mux4 #(parameter WIDTH = 8) (
  input  logic [WIDTH-1:0] d0, d1, d2, d3,
  input  logic [1:0]       s, 
  output logic [WIDTH-1:0] y);

  assign y = s[1] ? (s[0] ? d3 : d2) : (s[0] ? d1 : d0); 
endmodule

module mux5 #(parameter WIDTH = 8) (
  input  logic [WIDTH-1:0] d0, d1, d2, d3, d4,
  input  logic [2:0]       s, 
  output logic [WIDTH-1:0] y);

  assign y = s[2] ? d4 : (s[1] ? (s[0] ? d3 : d2) : (s[0] ? d1 : d0)); 
endmodule

module mux6 #(parameter WIDTH = 8) (
  input  logic [WIDTH-1:0] d0, d1, d2, d3, d4, d5,
  input  logic [2:0]       s, 
  output logic [WIDTH-1:0] y);

  assign y = s[2] ? (s[0] ? d5 : d4) : (s[1] ? (s[0] ? d3 : d2) : (s[0] ? d1 : d0)); 
endmodule

module imem (input  logic [31:0] a,
	     output logic [31:0] rd);
   
   logic [31:0] 		 RAM[63:0];
   
   assign rd = RAM[a[31:2]]; // word aligned
   
endmodule // imem

module dmem (input  logic        clk, we,
	     input  logic [31:0] a, wd,
	     output logic [31:0] rd);
   
   logic [31:0] 		 RAM[255:0];
   
   assign rd = RAM[a[31:2]]; // word aligned
   always_ff @(posedge clk)
     if (we) RAM[a[31:2]] <= wd;
   
endmodule // dmem

/*

module regfile(input  logic        clk, 
               input logic 	   we3, 
               input logic [ 4:0]  a1, a2, a3, 
               input logic [31:0]  wd3, 
               output logic [31:0] rd1, rd2);

   logic [31:0] 		   rf[31:0];

   // three ported register file
   // read two ports combinationally (A1/RD1, A2/RD2)
   // write third port on rising edge of clock (A3/WD3/WE3)
   // write occurs on falling edge of clock
   // register 0 hardwired to 0

   always_ff @(negedge clk)
     if (we3) rf[a3] <= wd3;	

   assign rd1 = (a1 != 0) ? rf[a1] : 0;
   assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule
*/
module adder(input  [31:0] a, b,
             output [31:0] y);

   assign y = a + b;
endmodule


module hazard(input  logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
              input logic 	 PCSrcE, ResultSrcEb0, 
              input logic 	 RegWriteM, RegWriteW,
              output logic [1:0] ForwardAE, ForwardBE,
              output logic 	 StallF, StallD, FlushD, FlushE);

   logic 			 lwStallD;
   
   // forwarding logic
   always_comb begin
      ForwardAE = 2'b00;
      ForwardBE = 2'b00;
      if (Rs1E != 5'b0)
	if      ((Rs1E == RdM) & RegWriteM) ForwardAE = 2'b10;
	else if ((Rs1E == RdW) & RegWriteW) ForwardAE = 2'b01;
      
      if (Rs2E != 5'b0)
	if      ((Rs2E == RdM) & RegWriteM) ForwardBE = 2'b10;
	else if ((Rs2E == RdW) & RegWriteW) ForwardBE = 2'b01;
   end
   
   // stalls and flushes
   assign lwStallD = ResultSrcEb0 & ((Rs1D == RdE) | (Rs2D == RdE));  
   assign StallD = lwStallD;
   assign StallF = lwStallD;
   assign FlushD = PCSrcE;
   assign FlushE = lwStallD | PCSrcE;
endmodule
