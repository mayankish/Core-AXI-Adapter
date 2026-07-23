// ============================================================
// Testbench  : tb_core_axi_adapter_with_core
// Project    : core_axi_adapter - Tesla AI Hardware Portfolio
// Author     : Mayank
// Description: Integration smoke test - the REAL riscv_ooo_top (not a
//              synthetic driver) talking to core_axi_adapter, talking to
//              the same multi-cycle AXI4-Lite behavioral slave used in
//              tb_core_axi_adapter.v. Both sub-pieces are already
//              independently verified (riscv_ooo_core's own testbenches;
//              the adapter's own tb_core_axi_adapter.v after the
//              dmem_ready-lag fix) - this test exists only to prove the
//              WIRING between them is correct (port widths/handshake
//              actually line up end to end), not to re-derive either
//              piece's internal correctness.
//
//              Program: store x2(=77) to address x1(=3), then load it
//              back into x4, then halt. Passes if x4 commits as 77 and
//              the slave's backing memory at word 3 reads 77 directly.
// ============================================================

`timescale 1ns/1ps
`include "defines.vh"

module tb_core_axi_adapter_with_core;

localparam IMEM_AW = 8;
localparam DMEM_AW = 8;   // must match core_axi_adapter's AW below
localparam IMEM_WORDS = (1 << IMEM_AW);
localparam TIMEOUT_CYCLES = 300;

reg clk, rst_n, flush, wake_i;

wire [IMEM_AW-1:0] imem_addr;
reg  [31:0]        imem [0:IMEM_WORDS-1];
wire [31:0]        imem_rdata = imem[imem_addr];

wire [DMEM_AW-1:0] dmem_addr;
wire               dmem_req, dmem_we;
wire [31:0]        dmem_wdata, dmem_rdata;
wire               dmem_ready;

wire commit_valid;
wire [4:0] commit_rd;
wire [31:0] commit_data;
wire [3:0] commit_tag;
wire commit_fault;
wire [31:0] pc_out;
wire fetch_valid, is_ext_out;
wire [2:0] ext_class_out;
wire core_halted, core_faulted, core_sleeping, core_clk_en;
wire rs_full, rob_full, pipeline_busy;

riscv_ooo_top #(.IMEM_AW(IMEM_AW), .DMEM_AW(DMEM_AW)) core (
    .clk(clk), .rst_n(rst_n), .flush(flush), .wake_i(wake_i),
    .imem_addr(imem_addr), .imem_rdata(imem_rdata),
    .dmem_addr(dmem_addr), .dmem_req(dmem_req), .dmem_we(dmem_we),
    .dmem_wdata(dmem_wdata), .dmem_rdata(dmem_rdata), .dmem_ready(dmem_ready),
    .commit_valid(commit_valid), .commit_rd(commit_rd), .commit_data(commit_data),
    .commit_tag(commit_tag), .commit_fault(commit_fault),
    .pc_out(pc_out), .fetch_valid(fetch_valid),
    .is_ext_out(is_ext_out), .ext_class_out(ext_class_out),
    .core_halted(core_halted), .core_faulted(core_faulted),
    .core_sleeping(core_sleeping), .core_clk_en(core_clk_en),
    .rs_full(rs_full), .rob_full(rob_full), .pipeline_busy(pipeline_busy)
);

wire [DMEM_AW-1:0] m_awaddr, m_araddr;
wire m_awvalid, m_awready, m_wvalid, m_wready, m_bvalid, m_bready;
wire m_arvalid, m_arready, m_rvalid, m_rready;
wire [31:0] m_wdata, m_rdata;
wire [3:0] m_wstrb;
wire [1:0] m_bresp, m_rresp;

core_axi_adapter #(.AW(DMEM_AW), .DW(32)) adapter (
    .clk(clk), .rst_n(rst_n),
    .dmem_addr(dmem_addr), .dmem_req(dmem_req), .dmem_we(dmem_we),
    .dmem_wdata(dmem_wdata), .dmem_rdata(dmem_rdata), .dmem_ready(dmem_ready),
    .m_awaddr(m_awaddr), .m_awvalid(m_awvalid), .m_awready(m_awready),
    .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wvalid(m_wvalid), .m_wready(m_wready),
    .m_bresp(m_bresp), .m_bvalid(m_bvalid), .m_bready(m_bready),
    .m_araddr(m_araddr), .m_arvalid(m_arvalid), .m_arready(m_arready),
    .m_rdata(m_rdata), .m_rresp(m_rresp), .m_rvalid(m_rvalid), .m_rready(m_rready)
);

axi4l_behavioral_slave #(.AW(DMEM_AW), .DW(32), .AW_DELAY(2), .B_DELAY(1), .R_DELAY(2)) slave (
    .clk(clk), .rst_n(rst_n),
    .s_awaddr(m_awaddr), .s_awvalid(m_awvalid), .s_awready(m_awready),
    .s_wdata(m_wdata), .s_wstrb(m_wstrb), .s_wvalid(m_wvalid), .s_wready(m_wready),
    .s_bresp(m_bresp), .s_bvalid(m_bvalid), .s_bready(m_bready),
    .s_araddr(m_araddr), .s_arvalid(m_arvalid), .s_arready(m_arready),
    .s_rdata(m_rdata), .s_rresp(m_rresp), .s_rvalid(m_rvalid), .s_rready(m_rready)
);

initial clk = 1'b0;
always #5 clk = ~clk;

initial begin
    $dumpfile("sim/tb_core_axi_adapter_with_core.vcd");
    $dumpvars(0, tb_core_axi_adapter_with_core);
end

function [31:0] f_alu;
    input [4:0] rd, rs1, rs2;
    input [2:0] op;
    f_alu = {rd, rs1, rs2, op, 14'b0};
endfunction
function [31:0] f_addi;
    input [4:0] rd, rs1;
    input [10:0] imm;
    f_addi = {rd, rs1, 5'b0, `OP_EXT, `EXTC_ADDI, imm};
endfunction
function [31:0] f_store;
    input [4:0] rs1, rs2;
    input [10:0] offset;
    f_store = {5'b0, rs1, rs2, `OP_EXT, `EXTC_STORE, offset};
endfunction
function [31:0] f_load;
    input [4:0] rd, rs1;
    input [10:0] offset;
    f_load = {rd, rs1, 5'b0, `OP_EXT, `EXTC_LOAD, offset};
endfunction
function [31:0] f_system;
    input [10:0] payload;
    f_system = {5'b0, 5'b0, 5'b0, `OP_EXT, `EXTC_SYSTEM, payload};
endfunction

integer i, errors, cyc;
reg x4_checked;

initial begin
    for (i = 0; i < IMEM_WORDS; i = i + 1) imem[i] = 32'h0;
    imem[0] = f_addi(5'd1, 5'd0, 11'd3);     // x1 = 3 (address)
    imem[1] = f_addi(5'd2, 5'd0, 11'd77);    // x2 = 77 (value)
    imem[2] = f_store(5'd1, 5'd2, 11'd0);    // dmem[3] = 77 (through the adapter!)
    imem[3] = f_addi(5'd3, 5'd0, 11'd0);     // filler
    imem[4] = f_load(5'd4, 5'd1, 11'd0);     // x4 = dmem[3] (through the adapter!)
    imem[5] = f_system(`SYS_HALT);
end

initial begin
    rst_n = 1'b0; flush = 1'b0; wake_i = 1'b0;
    errors = 0; cyc = 0; x4_checked = 1'b0;
    repeat (4) @(negedge clk);
    rst_n = 1'b1;
end

always @(posedge clk) begin
    if (rst_n) begin
        cyc <= cyc + 1;

        if (commit_fault) begin
            $display("[%0t] ERROR: commit_fault asserted", $time);
            errors = errors + 1;
        end

        if (commit_valid && (commit_rd == 5'd4)) begin
            x4_checked = 1'b1;
            if ($signed(commit_data) !== 32'sd77) begin
                $display("[%0t] ERROR: x4 = %0d, expected 77", $time, $signed(commit_data));
                errors = errors + 1;
            end else begin
                $display("[%0t] OK: x4 = 77 (round-tripped through core_axi_adapter + AXI slave)", $time);
            end
        end

        if (core_halted && x4_checked) begin
            $display("==============================================");
            if (slave.mem[3] !== 32'd77) begin
                $display("[%0t] ERROR: slave.mem[3] = %0d, expected 77", $time, slave.mem[3]);
                errors = errors + 1;
            end else begin
                $display("[%0t] OK: slave backing memory[3] = 77 directly", $time);
            end
            if (errors == 0)
                $display("TB_CORE_AXI_ADAPTER_WITH_CORE: PASS (0 errors)");
            else
                $display("TB_CORE_AXI_ADAPTER_WITH_CORE: FAIL (%0d errors)", errors);
            $display("==============================================");
            $finish;
        end

        if (cyc >= TIMEOUT_CYCLES) begin
            $display("[%0t] ERROR: TIMEOUT, x4_checked=%0b core_halted=%0b", $time, x4_checked, core_halted);
            $display("TB_CORE_AXI_ADAPTER_WITH_CORE: FAIL (timeout)");
            $finish;
        end
    end
end

endmodule
