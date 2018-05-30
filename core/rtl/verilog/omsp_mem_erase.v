//----------------------------------------------------------------------------
// Copyright (C) 2015 Johannes Goetzfried
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the authors nor the names of its contributors
//       may be used to endorse or promote products derived from this software
//       without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
// OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE
//
//----------------------------------------------------------------------------
//
// *File Name: omsp_mem_erase.v
//
// *Module Description:
//                       Wipe data memory after reset
//
// *Author(s):
//              - Ruan de Clercq,      ruan.declercq@esat.kuleuven.be
//              - Johannes Goetzfried, johannes.goetzfried@cs.fau.de
//
//----------------------------------------------------------------------------
`ifdef OMSP_NO_INCLUDE
`else
`include "openMSP430_defines.v"
`endif

module omsp_mem_erase (

// OUTPUTs
    dmem_addr,                      // Data Memory address
    dmem_cen,                       // Data Memory chip enable (low active)
    dmem_din,                       // Data Memory data input
    dmem_wen,                       // Data Memory write enable (low active)
    puc_rst,                        // Emulated main system reset

// INPUTs
    mem_dmem_addr,                  // Data Memory address
    mem_dmem_cen,                   // Data Memory chip enable (low active)
    mem_dmem_din,                   // Data Memory data input
    mem_dmem_wen,                   // Data Memory write enable (low active)
    mclk,                           // Main system clock
    cpuoff,                         // Turns off the CPU
    clk_puc_rst                     // Main system reset from clock unit
);

// OUTPUTs
//=========
output [`DMEM_MSB:0] dmem_addr;     // Data Memory address
output               dmem_cen;      // Data Memory chip enable (low active)
output        [15:0] dmem_din;      // Data Memory data input
output         [1:0] dmem_wen;      // Data Memory write enable (low active)
output               puc_rst;       // Emulated main system reset

// INPUTs
//=========
input  [`DMEM_MSB:0] mem_dmem_addr; // Data Memory address
input                mem_dmem_cen;  // Data Memory chip enable (low active)
input         [15:0] mem_dmem_din;  // Data Memory data input
input          [1:0] mem_dmem_wen;  // Data Memory write enable (low active)
input                mclk;          // Main system clock
input                cpuoff;        // Turns off the CPU
input                clk_puc_rst;   // Main system reset from clock unit


//=============================================================================

`define I_FORWARD    2'h0 // Normal operation. Erasing is not happening
`define I_ERASE      2'h1 // Busy performing erase
`define I_RESETCNT   2'h2 // Reset the counter and start erasing

reg                  puc_rst;
reg    [`DMEM_MSB:0] cnt_value;
reg            [1:0] i_state;
reg            [1:0] i_state_nxt;

wire                 erase_complete;
wire                 cnt_reset;


//=============================================================================

// puc_rst keeps the MCU in a reset state while the erasing is happening
always @ (cpuoff, clk_puc_rst, erase_complete)
begin
    if (clk_puc_rst)
        puc_rst = ~cpuoff & clk_puc_rst;
    else
        puc_rst = ~erase_complete;
end

// Finite state machine
always @ (*)
    case (i_state)
        `I_RESETCNT : i_state_nxt = `I_ERASE;
        `I_ERASE    : i_state_nxt = (cnt_value == (`DMEM_SIZE>>1))
                                    ? `I_FORWARD : `I_ERASE;
        `I_FORWARD  : i_state_nxt = (puc_rst) ? `I_ERASE : `I_FORWARD;
    endcase

// Finite state machine
always @ (posedge mclk or posedge clk_puc_rst)
    if (clk_puc_rst) i_state <= `I_RESETCNT;
    else             i_state <= i_state_nxt;

// Multiplex between the memory erase signals and the dmem signals of the MCU
assign dmem_addr = ~erase_complete ? cnt_value : mem_dmem_addr;
assign dmem_cen  = ~erase_complete ?      1'b0 : mem_dmem_cen;
assign dmem_din  = ~erase_complete ?     16'b0 : mem_dmem_din;
assign dmem_wen  = ~erase_complete ?      2'b0 : mem_dmem_wen;

assign erase_complete = (i_state == `I_FORWARD);

// Reset the counter when inside the I_RESETCNT state
assign cnt_reset = (i_state == `I_RESETCNT);
always @ (posedge mclk, posedge cnt_reset)
begin
    if (cnt_reset)
        cnt_value <= 0;
    else
        cnt_value <= cnt_value + 1;
end
    
endmodule
