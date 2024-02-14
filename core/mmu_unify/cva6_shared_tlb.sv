// Copyright (c) 2023 Thales.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Angela Gonzalez PlanV Technology
// Date: 24/11/2023
//
// Description: N-way associative shared TLB, it allows to reduce the number
//              of ITLB and DTLB entries.
//
// =========================================================================== //
// Revisions  :
// Date        Version  Author       Description
// 2024-02-05  0.2      A.Gonzalez   Generic shared TLB for CVA6 with Hypervisor support
// =========================================================================== //

/* verilator lint_off WIDTH */

module cva6_shared_tlb
#(
  parameter type pte_cva6_t = logic,
  parameter type tlb_update_cva6_t = logic,
  parameter int SHARED_TLB_DEPTH = 64,
  parameter int SHARED_TLB_WAYS = 2,
  parameter int unsigned HYP_EXT = 0,
  parameter int unsigned ASID_WIDTH [HYP_EXT:0] = {1}, //[vmid_width,asid_width]
  parameter int unsigned ASID_LEN = 1,
  parameter int unsigned VPN_LEN = 1,
  parameter int unsigned PT_LEVELS = 1
) (
    input logic clk_i,   // Clock
    input logic rst_ni,  // Asynchronous reset active low
    input  logic   [HYP_EXT*2:0]    flush_i,  // Flush signal [g_stage,vs stage, normal translation signal]
    input  logic   [1:0][HYP_EXT*2:0] v_st_enbl_i,  // v_i,g-stage enabled, s-stage enabled
    input  logic   [HYP_EXT*2:0]    enable_translation_i, //[v_i,enable_g_translation,enable_translation]
    input  logic   [HYP_EXT*2:0]    en_ld_st_translation_i,   // enable virtual memory translation for load/stores


    input logic [ASID_WIDTH[0]-1:0] dtlb_asid_i[HYP_EXT:0],//[vmid,vs_asid,asid]
    input logic [ASID_WIDTH[0]-1:0] itlb_asid_i[HYP_EXT:0],//[vmid,vs_asid,asid]

    // from TLBs
    // did we miss?
    input logic                   itlb_access_i,
    input logic                   itlb_hit_i,
    input logic [riscv::VLEN-1:0] itlb_vaddr_i,

    input logic                   dtlb_access_i,
    input logic                   dtlb_hit_i,
    input logic [riscv::VLEN-1:0] dtlb_vaddr_i,

    // to TLBs, update logic
    output tlb_update_cva6_t itlb_update_o,
    output tlb_update_cva6_t dtlb_update_o,

    // Performance counters
    output logic itlb_miss_o,
    output logic dtlb_miss_o,

    output logic                   shared_tlb_access_o,
    output logic                   shared_tlb_hit_o,
    output logic [riscv::VLEN-1:0] shared_tlb_vaddr_o,

    output logic itlb_req_o,

    // Update shared TLB in case of miss
    input tlb_update_cva6_t shared_tlb_update_i

);

tlb_update_cva6_t shared_tlb_update_delayed,shared_tlb_update_delayed2;
logic shared_tlb_update_valid_delayed,shared_tlb_update_valid_delayed2;

function logic [SHARED_TLB_WAYS-1:0] shared_tlb_way_bin2oh(input logic [$clog2(SHARED_TLB_WAYS
)-1:0] in);
  logic [SHARED_TLB_WAYS-1:0] out;
  out     = '0;
  out[in] = 1'b1;
  return out;
endfunction

typedef struct packed {
logic [HYP_EXT:0][ASID_LEN-1:0]                        asid;   
logic [PT_LEVELS+HYP_EXT-1:0][(VPN_LEN/PT_LEVELS)-1:0] vpn;   
logic [PT_LEVELS-2:0][HYP_EXT:0]                       is_page;
logic [HYP_EXT*2:0]                                    v_st_enbl; // v_i,g-stage enabled, s-stage enabled
} shared_tag_t;

shared_tag_t shared_tag_wr;
shared_tag_t [SHARED_TLB_WAYS-1:0] shared_tag_rd;

logic [SHARED_TLB_DEPTH-1:0][SHARED_TLB_WAYS-1:0] shared_tag_valid_q, shared_tag_valid_d;

logic [         SHARED_TLB_WAYS-1:0] shared_tag_valid;

logic [         SHARED_TLB_WAYS-1:0] tag_wr_en;
logic [$clog2(SHARED_TLB_DEPTH)-1:0] tag_wr_addr;
logic [     $bits(shared_tag_t)-1:0] tag_wr_data;

logic [         SHARED_TLB_WAYS-1:0] tag_rd_en;
logic [$clog2(SHARED_TLB_DEPTH)-1:0] tag_rd_addr;
logic [     $bits(shared_tag_t)-1:0] tag_rd_data      [SHARED_TLB_WAYS-1:0];

logic [         SHARED_TLB_WAYS-1:0] tag_req;
logic [         SHARED_TLB_WAYS-1:0] tag_we;
logic [$clog2(SHARED_TLB_DEPTH)-1:0] tag_addr;

logic [         SHARED_TLB_WAYS-1:0] pte_wr_en;
logic [$clog2(SHARED_TLB_DEPTH)-1:0] pte_wr_addr;
logic [$bits(pte_cva6_t)-1:0] pte_wr_data [HYP_EXT:0];

logic [         SHARED_TLB_WAYS-1:0] pte_rd_en;
logic [$clog2(SHARED_TLB_DEPTH)-1:0] pte_rd_addr;
logic [$bits(pte_cva6_t)-1:0] pte_rd_data      [SHARED_TLB_WAYS-1:0][HYP_EXT:0];

logic [         SHARED_TLB_WAYS-1:0] pte_req;
logic [         SHARED_TLB_WAYS-1:0] pte_we;
logic [$clog2(SHARED_TLB_DEPTH)-1:0] pte_addr;

logic [PT_LEVELS+HYP_EXT-1:0][(VPN_LEN/PT_LEVELS)-1:0] vpn_d,vpn_q;   
logic [SHARED_TLB_WAYS-1:0][PT_LEVELS-1:0] vpn_match;
logic [SHARED_TLB_WAYS-1:0][PT_LEVELS-1:0] page_match;
logic [SHARED_TLB_WAYS-1:0][PT_LEVELS-1:0] level_match;

logic [SHARED_TLB_WAYS-1:0][HYP_EXT:0] match_asid;
logic [SHARED_TLB_WAYS-1:0] match_stage;

pte_cva6_t [SHARED_TLB_WAYS-1:0][HYP_EXT:0] pte;
pte_cva6_t  g_content;
logic [riscv::GPLEN-1:0] lu_gpaddr;

logic [riscv::VLEN-1-12:0] itlb_vpn_q;
logic [riscv::VLEN-1-12:0] dtlb_vpn_q;

logic [ASID_WIDTH[0]-1:0] tlb_update_asid_q [HYP_EXT:0], tlb_update_asid_d[HYP_EXT:0];

logic shared_tlb_access_q, shared_tlb_access_d;
logic shared_tlb_hit_d;
logic [riscv::VLEN-1:0] shared_tlb_vaddr_q, shared_tlb_vaddr_d;

logic itlb_req_d, itlb_req_q;
logic dtlb_req_d, dtlb_req_q;

int i_req;

// replacement strategy
logic [SHARED_TLB_WAYS-1:0] way_valid;
logic update_lfsr;  // shift the LFSR
logic [$clog2(SHARED_TLB_WAYS)-1:0] inv_way;  // first non-valid encountered
logic [$clog2(SHARED_TLB_WAYS)-1:0] rnd_way;  // random index for replacement
logic [$clog2(SHARED_TLB_WAYS)-1:0] repl_way;  // way to replace
logic [SHARED_TLB_WAYS-1:0] repl_way_oh_d;  // way to replace (onehot)
logic all_ways_valid;  // we need to switch repl strategy since all are valid

assign shared_tlb_access_o = shared_tlb_access_q;
assign shared_tlb_hit_o = shared_tlb_hit_d;
assign shared_tlb_vaddr_o = shared_tlb_vaddr_q;

assign itlb_req_o = itlb_req_q;

   
///////////////////////////////////////////////////////
// tag comparison, hit generation
///////////////////////////////////////////////////////
    always_comb begin : itlb_dtlb_miss
        itlb_miss_o         = 1'b0;
        dtlb_miss_o         = 1'b0;

        itlb_req_d          = 1'b0;
        dtlb_req_d          = 1'b0;

        shared_tlb_access_d = '0;
        shared_tlb_vaddr_d  = shared_tlb_vaddr_q;
    
 // if we got an ITLB miss
        if ((|enable_translation_i[HYP_EXT:0]) & itlb_access_i & ~itlb_hit_i & ~dtlb_access_i) begin      
            shared_tlb_access_d = '1;
            shared_tlb_vaddr_d  = itlb_vaddr_i;
            itlb_req_d          = 1'b1;
            itlb_miss_o         = 1'b1;

            // we got an DTLB miss
          end else if ((|en_ld_st_translation_i[HYP_EXT:0]) & dtlb_access_i & ~dtlb_hit_i) begin
            shared_tlb_access_d = '1;
            shared_tlb_vaddr_d  = dtlb_vaddr_i;
            dtlb_req_d          = 1'b1;
            dtlb_miss_o         = 1'b1;       
          end
        end  //itlb_dtlb_miss
        always_comb begin : tag_comparison
            shared_tlb_hit_d = 1'b0;
            dtlb_update_o = '0;
            itlb_update_o = '0;

        // for (int unsigned i = 0; i < SHARED_TLB_WAYS; i++) begin
            if(shared_tlb_update_i.valid) begin
                shared_tlb_hit_d = 1'b1;
                if (itlb_req_q) begin
                    itlb_update_o.valid = 1'b1;
                    itlb_update_o.vpn = shared_tlb_update_i.vpn;
                    itlb_update_o.is_page = shared_tlb_update_i.is_page;
                    itlb_update_o.content = shared_tlb_update_i.content;
                    itlb_update_o.asid = shared_tlb_update_i.asid;
                    
                end else if (dtlb_req_q) begin
                    dtlb_update_o.valid = 1'b1;
                    dtlb_update_o.vpn = shared_tlb_update_i.vpn;
                    dtlb_update_o.is_page = shared_tlb_update_i.is_page;
                    dtlb_update_o.content = shared_tlb_update_i.content;
                    dtlb_update_o.asid = shared_tlb_update_i.asid;
                end
            end
        end //tag_comparison

        // sequential process
        always_ff @(posedge clk_i or negedge rst_ni) begin
            if (~rst_ni) begin
                shared_tlb_update_valid_delayed<='0;
                shared_tlb_access_q <= '0;
                shared_tlb_vaddr_q <= '0;
                itlb_req_q <= '0;
                dtlb_req_q <= '0;
                shared_tlb_update_valid_delayed2<='0;
                shared_tlb_update_delayed<='0;
                shared_tlb_update_delayed2<='0;
            end else begin
                shared_tlb_access_q <= shared_tlb_access_d;
                shared_tlb_vaddr_q <= shared_tlb_vaddr_d;
                itlb_req_q <= itlb_req_d;
                dtlb_req_q <= dtlb_req_d;
                
                shared_tlb_update_valid_delayed <=shared_tlb_update_i.valid;
                shared_tlb_update_delayed <= shared_tlb_update_i;

                shared_tlb_update_valid_delayed2 <=  shared_tlb_update_valid_delayed2 && shared_tlb_access_q ? shared_tlb_update_valid_delayed2 : shared_tlb_update_valid_delayed;
                shared_tlb_update_delayed2 <= shared_tlb_update_valid_delayed2 && shared_tlb_access_q ? shared_tlb_update_delayed2 : shared_tlb_update_delayed;
            end
        end

endmodule

/* verilator lint_on WIDTH */