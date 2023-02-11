//////////////////////////////////////////////////////////////////////////////
// SPDX-FileCopyrightText: 2021 , Dinesh Annayya                          
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0
// SPDX-FileContributor: Created by Dinesh Annayya <dinesha@opencores.org>
//
//////////////////////////////////////////////////////////////////////
////                                                              ////
////  QSPIS Interface                                            ////
////                                                              ////
////  This file is part of the riscduino cores project            ////
////  https://github.com/dineshannayya/riscduino.git              ////
////                                                              ////
////  Description : This module contains SPI interface            ////
////                 state machine                                ////
////                                                              ////   
////  To Do:                                                      ////
////    nothing                                                   ////
////                                                              ////
////  Author(s):                                                  ////
////      - Dinesh Annayya, dinesh.annayya@gmail.com              ////
////                                                              ////
////  Revision :                                                  ////
////    0.1 - 11th Feb 2023, Dinesh A                            ////
////          Initial version                                     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
/*********************************************************************
   CMD Decoding [7:0]
             [7:4] = 4'b1 - READ  REGISTER
                   = 4'b2 - WRITE REGISTER
             [3:0] = Byte Enable valid only during Write Command
*********************************************************************/

module qspis_if (

	     input  logic         sys_clk         ,
	     input  logic         rst_n           ,

             input  logic         sclk            ,
             input  logic         ssn             ,
             input  logic [3:0]   sdin            ,
             output logic [3:0]   sdout           ,
             output logic         sdout_oen       ,

             //spi_sm Interface
             output logic         reg_wr          , // write request
             output logic         reg_rd          , // read request
             output logic [31:0]  reg_addr        , // address
             output logic  [3:0]  reg_be          , // Byte enable
             output logic [31:0]  reg_wdata       , // write data
             input  logic [31:0]  reg_rdata       , // read data
             input  logic         reg_ack           // read valid
             );


//--------------------------------------------------------
// Wire and reg definitions
// -------------------------------------------------------

reg  [5:0]     bitcnt           ;
reg  [7:0]     cmd_reg          ;
reg  [31:0]    RegSdOut         ;
reg [2:0]      spi_if_st        ;

parameter    idle_st    = 3'b000,
             cmd_st     = 3'b001,
             adr_st     = 3'b010,
             wr_st      = 3'b011,
             wwait_st   = 3'b100,
             rwait_st   = 3'b101,
             rd_st      = 3'b110;

parameter C_WREN        = 8'h06, // Write Enable
	  C_WRDS        = 8'h04, // Write Disable
	  C_RSR1        = 8'h05, // Read Status Reg-1
	  C_RSR2        = 8'h35, // Read Status Reg-2
	  C_RSR3        = 8'h15, // Read Status Reg-3
	  C_WSR1        = 8'h01, // Write Status Reg-1
	  C_WSR2        = 8'h31, // Write Status Reg-2
	  C_WSR3        = 8'h11, // Write Status Reg-3
	  C_RD          = 8'h03, // Read Data
	  C_FRD         = 8'h0B, // Fast Read Data
	  C_FDRD        = 8'h3B, // Fast Dual Read
	  C_FRDIO       = 8'hBB, // Fast Dual IO Read
	  C_FRQIO       = 8'hEB, // Fast Quad IO Read
	  C_PWDN        = 8'hB9, // Power Down
	  C_RMID        = 8'h90, // Read Manufacture ID
	  C_RMIDDIO     = 8'h92, // Read Manufacture ID, Dual -IO
	  C_RMIDQIO     = 8'h94, // Read Manufacture ID, Quad -IO
	  C_RJEDEC      = 8'h9F, // Read JEDEC ID
	  C_PPGM        = 8'h02, // Page Write
	  C_QPPGM       = 8'h32, // Quad Page Write
	  WRITE_CMD     = 4'h2;
    

wire adr_phase     = (spi_if_st == adr_st);
wire cmd_phase     = (spi_if_st == cmd_st);
wire wr_phase      = (spi_if_st == wr_st);
wire rd_phase      = (spi_if_st == rd_st);
wire cnt_phase     = (spi_if_st != wwait_st) && (spi_if_st != rwait_st);
wire wwait_phase   = (spi_if_st == wwait_st);
wire rwait_phase   = (spi_if_st == rwait_st);




// sclk pos and ned edge generation
logic     sck_l0,sck_l1,sck_l2;

wire sck_pdetect = (!sck_l2 && sck_l1) ? 1'b1: 1'b0;
wire sck_ndetect = (sck_l2 && !sck_l1) ? 1'b1: 1'b0;

always @ (posedge sys_clk or negedge rst_n) begin
if (!rst_n) begin
      sck_l0 <= 1'b1;
      sck_l1 <= 1'b1;
      sck_l2 <= 1'b1;
   end
   else begin
      sck_l0 <= sclk;
      sck_l1 <= sck_l0; // double sync
      sck_l2 <= sck_l1;
   end
end

// SSN double sync
logic     ssn_l0,ssn_l1, ssn_ss;

assign ssn_ss = ssn_l1;

always @ (posedge sys_clk or negedge rst_n) begin
if (!rst_n) begin
      ssn_l0 <= 1'b1;
      ssn_l1 <= 1'b1;
   end
   else begin
      ssn_l0 <= ssn;
      ssn_l1 <= ssn_l0; // double sync
   end
end


//command register accumation
assign reg_be = cmd_reg[3:0];

always @(negedge rst_n or posedge sys_clk)
begin
  if (!rst_n)
     cmd_reg[7:0] <= 8'b0;
  else if (cmd_phase & (sck_pdetect))
     cmd_reg[7:0] <= {cmd_reg[6:0], sdin};
end


// address accumation at posedge sclk
always @(negedge rst_n or posedge sys_clk)
begin
  if (!rst_n)
     reg_addr[31:0] <= 32'b0;
  else if (adr_phase & (sck_pdetect))
     reg_addr[31:0] <= {reg_addr[30:0], sdin};
end 

// write data accumation at posedge sclk
always @(negedge rst_n or posedge sys_clk)
begin
  if (!rst_n)
     reg_wdata[31:0] <= 32'b0;
  else if (wr_phase & (sck_pdetect))
     reg_wdata[31:0] <= {reg_wdata[30:0], sdin};
end



// drive sdout at negedge sclk 
always @(negedge rst_n or posedge sys_clk)
begin
  if (!rst_n) begin
     RegSdOut[31:0] <= 32'b0;
     sdout          <= 1'b0;
  end else begin
      if (reg_ack)
          RegSdOut <= reg_rdata[31:0];
      else if (rd_phase && sck_ndetect)
          RegSdOut <= {RegSdOut[30:0], 1'b0};

     sdout <= (rd_phase && sck_ndetect) ? RegSdOut[31] : sdout;
   end
end


// SPI State Machine
always @(negedge rst_n or posedge sys_clk)
begin
   if (!rst_n) begin
            reg_wr       <= 1'b0;
            reg_rd       <= 1'b0;
            sdout_oen    <= 1'b1;
            bitcnt       <= 6'b0;
	    cfg_wren     <= 1'b0;
            spi_if_st    <= idle_st;
   end else if(ssn_ss)    begin
            reg_wr       <= 1'b0;
            reg_rd       <= 1'b0;
            sdout_oen    <= 1'b1;
            bitcnt       <= 6'b0;
	    spi_if_st    <= idle_st; 
   end else begin
       case (spi_if_st)
          idle_st  : begin // Idle State
             reg_wr       <= 1'b0;
             reg_rd       <= 1'b0;
             sdout_oen    <= 1'b1;
             bitcnt       <= 6'b0;
             if (ssn_ss == 1'b0) begin
                spi_if_st <= cmd_st;
             end 
          end

          cmd_st : begin // Command State
             if (ssn_ss == 1'b1) begin
                spi_if_st <= idle_st;
            end else if (sck_pdetect) begin
                if(bitcnt   == 6'b000111)  begin
                    bitcnt     <= 6'b0;
                    spi_if_st  <= adr_st;
                end else begin
                    bitcnt       <= bitcnt  +1;
                end
             end
           end

          adr_st : begin // Address Phase
             reg_wr       <= 1'b0;
             reg_rd       <= 1'b0;
             sdout_oen    <= 1'b1;
             if (ssn_ss == 1'b1) begin
                spi_if_st <= idle_st;
             end else if (sck_pdetect) begin
                if (bitcnt   == 6'b011111) begin
                   bitcnt    <= 6'b0;
		   case(cmd_reg)
		   C_WREN: begin // 0x06 Write Enable
		      cfg_wren  <= 1'b1;
                      spi_if_st <= exit_st;
		   end
		   C_WRDS: begin // 0x04 Write Disable
		      cfg_wren  <= 1'b0;
                      spi_if_st <= exit_st;
		   end
		   C_RSR1: begin // 0x05 - Read Status Register-1
                      spi_if_st <= rdstat1_st;
		   end
		   C_RSR2: begin // 0x35 - Read Status Register-2
                      spi_if_st <= rdstat2_st;
		   end
		   C_RSR3: begin // 0x15 - Read Status Register-3
                      spi_if_st <= rdstat3_st;
		   end
		   C_WSR1: begin // 0x01 - Write Status Register-1
                      spi_if_st <= wrstat1_st;
		   end
		   C_WSR2: begin // 0x31 - Write Status Register-2
                      spi_if_st <= wrstat2_st;
		   end
		   C_WSR3: begin // 0x11 - Write Status Register-3
                      spi_if_st <= wrstat3_st;
		   end
		   C_RD: begin // Read Data - 0x03
		      cfg_fast_rd <= 1'b0;
		      cfg_spi_phase <= P_READ; 
		      cfg_spi_amode <= P_SINGLE; // Address Single Bit Mode
		      cfg_spi_dmode <= P_SINGLE; // Data Single Bit Mode
                      spi_if_st <= addr_st;
		   end
		   C_FRD: begin // Fast Read Data - 0x0B
		      cfg_fast_rd <= 1'b1;
		      cfg_spi_phase <= P_READ; 
		      cfg_spi_amode <= P_SINGLE; // Address Single Bit Mode
		      cfg_spi_dmode <= P_SINGLE; // Data Single Bit Mode
                      spi_if_st <= addr_st;
		   end
		   C_FDRD: begin // Fast Dual Read Data - 0x3B
		      cfg_fast_rd <= 1'b1;
		      cfg_spi_phase <= P_READ; 
		      cfg_spi_amode <= P_SINGLE; // Address Single Bit Mode
		      cfg_spi_dmode <= P_DUAL;   // Data Dual Bit Mode
                      spi_if_st <= addr_st;
		   end
		   C_FQRD: begin // Fast Quad Read Data - 0x6B
		      cfg_fast_rd <= 1'b1;
		      cfg_spi_phase <= P_READ; 
		      cfg_spi_amode <= P_SINGLE; // Address Single Bit Mode
		      cfg_spi_dmode <= P_QUAD  ; // Data Dual Bit Mode
                      spi_if_st <= addr_st;
		   end
		   C_FRDIO: begin // Fast Read Dual IO - 0xBB
		      cfg_fast_rd <= 1'b1;
		      cfg_spi_phase <= P_READ; 
		      cfg_spi_amode <= P_DUAL; // Address Dual Bit Mode
		      cfg_spi_dmode <= P_DUAL;   // Data Dual Bit Mode
                      spi_if_st <= addr_st;
		   end
		   C_FRQIO: begin // Fast Read Quad IO - 0xEB
		      cfg_fast_rd <= 1'b1;
		      cfg_spi_phase <= P_READ; 
		      cfg_spi_amode <= P_QUAD; // Address Dual Bit Mode
		      cfg_spi_dmode <= P_QUAD;   // Data Dual Bit Mode
                      spi_if_st <= addr_st;
		   end
		   C_PWDN: begin // Power Down - 0xB9
                      spi_if_st <= exit_st;
		   end
		   C_RMID: begin // Read Manufacture Id
                      spi_if_st <= chipid_st;
		      cfg_spi_amode <= P_SINGLE; 
		      cfg_spi_dmode <= P_SINGLE;  
		   end
		   C_RMIDDIO: begin // Read Manufacture Id-Dual IO
                      spi_if_st <= chipid_st;
		      cfg_spi_amode <= P_DUAL; 
		      cfg_spi_dmode <= P_DUAL;  
		   end
		   C_RMIDQIO: begin // Read Manufacture Id-Quad IO
                      spi_if_st <= chipid_st;
		      cfg_spi_amode <= P_QUAD; 
		      cfg_spi_dmode <= P_QUAD;  
		   end
		   C_RJEDEC: begin // Read JEDEC ID
		   default: begin
                      spi_if_st <= chipid_st;
		      cfg_spi_amode <= P_SINGLE; 
		      cfg_spi_dmode <= P_SINGLE;  
		   end
		   C_PPGM: begin
		      cfg_spi_phase <= P_WRITE; 
		      cfg_spi_amode <= P_SINGLE; 
		      cfg_spi_dmode <= P_SINGLE;  
                      spi_if_st     <= addr_st;

	           end
		   C_QPPGM: begin
		      cfg_spi_phase <= P_WRITE; 
		      cfg_spi_amode <= P_SINGLE; 
		      cfg_spi_dmode <= P_QUAD;  
                      spi_if_st     <= addr_st;

	           end
		   default: begin
                      spi_if_st <= exit_st;
		   end
                end else begin
                    bitcnt       <= bitcnt  +1;
                end
             end
          end

          wr_st   : begin // Write State
             if (ssn_ss == 1'b1) begin
                spi_if_st <= idle_st;
             end else if (sck_pdetect) begin
                if (bitcnt   == 6'b011111) begin
                   bitcnt     <= 6'b0;
                   spi_if_st  <= wwait_st;
                   reg_wr     <= 1;
                end else begin
                   bitcnt     <= bitcnt  +1;
                end
             end
          end
          wwait_st  : begin // Register Bus Busy Check State
	     if(reg_ack) reg_wr       <= 0;
             if (ssn_ss == 1'b1) begin
                spi_if_st <= idle_st;
             end else if (sck_pdetect) begin
                if (bitcnt   == 6'b000111) begin
                   bitcnt       <= 6'b0;
                   spi_if_st    <= cmd_st;
                end else begin
                   bitcnt       <= bitcnt  +1;
                end
             end
          end

          rwait_st  : begin // Read Wait State
             if(reg_ack) reg_rd     <= 1'b0;
             if (ssn_ss == 1'b1) begin
                spi_if_st <= idle_st;
             end else if (sck_pdetect) begin
                if (bitcnt   == 6'b000111) begin
                   reg_rd     <= 1'b0;
                   bitcnt     <= 6'b0;
                   sdout_oen  <= 1'b0;
                   spi_if_st  <= rd_st;
                end else begin
                   bitcnt     <= bitcnt  +1;
                end
             end
          end

          rd_st : begin // Send Data to SPI 
             if (ssn_ss == 1'b1) begin
                spi_if_st <= idle_st;
             end else if (sck_pdetect) begin
                if (bitcnt   == 6'b011111) begin
                   bitcnt     <= 6'b0;
                   sdout_oen  <= 1'b1;
                   spi_if_st  <= cmd_st;
                end else begin
                   bitcnt       <= bitcnt  +1;
                end
             end
          end
          exit_st : begin // Wait for SSN = 
             if (ssn_ss == 1'b1) begin
                spi_if_st <= idle_st;
	     end
	  end

          default      : spi_if_st <= idle_st;
       endcase
   end
end

endmodule
