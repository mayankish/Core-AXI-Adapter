// ============================================================
// Testbench  : tb_core_axi_adapter
// Project    : core_axi_adapter - Tesla AI Hardware Portfolio
// Author     : Mayank
// Description: Drives the adapter's dmem-side port with a REGISTERED
//              request signal that mirrors riscv_ooo_core's lsu_frontend.v
//              req_outstanding_r EXACTLY (a clocked reg that sets on a new
//              request and clears via non-blocking assignment the cycle
//              AFTER dmem_ready is observed - never nudged mid-cycle by a
//              blocking assign), presented back-to-back with ZERO idle
//              cycles between requests - the strictest case for catching a
//              suspected bug: if the adapter's internal state returns to
//              idle in a way that overlaps with the core's request signal
//              still being asserted (leftover from the request that JUST
//              completed), it could spuriously start a second, unwanted
//              transaction using stale address/data.
//
//              An earlier version of this testbench drove dmem_req via a
//              task with a blocking assignment at a negedge - which
//              (correctly, per the negedge-timing lessons learned earlier
//              this session) avoids posedge races, but ALSO happens to
//              drop dmem_req half a cycle earlier than a real clocked
//              register would, silently avoiding the exact hazard window
//              this test exists to check. Fixed by driving dmem_req from
//              an actual posedge-clocked always block instead.
//
//              Sequence: WRITE addr=5<-100, WRITE addr=6<-200,
//              READ addr=5 (expect 100), READ addr=6 (expect 200), all
//              back-to-back. Independently counts every AW-accept and
//              AR-accept on the slave side and requires EXACTLY 2 of each
//              - a stray 3rd accept on either channel means a spurious
//              duplicate fired.
// ============================================================

`timescale 1ns/1ps

module tb_core_axi_adapter;

localparam AW = 16;
localparam DW = 32;

reg clk, rst_n;

wire [AW-1:0] dmem_addr;
wire          dmem_req;
wire          dmem_we;
wire [DW-1:0] dmem_wdata;
wire [DW-1:0] dmem_rdata;
wire          dmem_ready;

wire [AW-1:0] m_awaddr;
wire          m_awvalid, m_awready;
wire [DW-1:0] m_wdata;
wire [DW/8-1:0] m_wstrb;
wire          m_wvalid, m_wready;
wire [1:0]    m_bresp;
wire          m_bvalid, m_bready;
wire [AW-1:0] m_araddr;
wire          m_arvalid, m_arready;
wire [DW-1:0] m_rdata;
wire [1:0]    m_rresp;
wire          m_rvalid, m_rready;

core_axi_adapter #(.AW(AW), .DW(DW)) dut (
    .clk(clk), .rst_n(rst_n),
    .dmem_addr(dmem_addr), .dmem_req(dmem_req), .dmem_we(dmem_we),
    .dmem_wdata(dmem_wdata), .dmem_rdata(dmem_rdata), .dmem_ready(dmem_ready),
    .m_awaddr(m_awaddr), .m_awvalid(m_awvalid), .m_awready(m_awready),
    .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wvalid(m_wvalid), .m_wready(m_wready),
    .m_bresp(m_bresp), .m_bvalid(m_bvalid), .m_bready(m_bready),
    .m_araddr(m_araddr), .m_arvalid(m_arvalid), .m_arready(m_arready),
    .m_rdata(m_rdata), .m_rresp(m_rresp), .m_rvalid(m_rvalid), .m_rready(m_rready)
);

axi4l_behavioral_slave #(.AW(AW), .DW(DW), .AW_DELAY(2), .B_DELAY(1), .R_DELAY(2)) slave (
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
    $dumpfile("sim/tb_core_axi_adapter.vcd");
    $dumpvars(0, tb_core_axi_adapter);
end

// ---- Independent transaction counters (slave-side accepts) -------------
integer aw_accepts, ar_accepts;
always @(posedge clk) begin
    if (!rst_n) begin
        aw_accepts <= 0;
        ar_accepts <= 0;
    end else begin
        if (m_awvalid && m_awready) aw_accepts <= aw_accepts + 1;
        if (m_arvalid && m_arready) ar_accepts <= ar_accepts + 1;
    end
end

// ---- Fake-core driver: a REAL clocked register, exactly mirroring
//      lsu_frontend's req_outstanding_r (sets on a new queued request,
//      clears via non-blocking assign the cycle after dmem_ready, never
//      touched by a blocking assign) -------------------------------------
localparam NREQ = 4;
reg [AW-1:0] q_addr   [0:NREQ-1];
reg          q_we     [0:NREQ-1];
reg [DW-1:0] q_wdata  [0:NREQ-1];
reg          q_is_read[0:NREQ-1];
reg [DW-1:0] q_expect [0:NREQ-1];
integer q_head;

reg          dmem_req_r;
reg [AW-1:0] dmem_addr_r;
reg          dmem_we_r;
reg [DW-1:0] dmem_wdata_r;

assign dmem_req   = dmem_req_r;
assign dmem_addr  = dmem_addr_r;
assign dmem_we    = dmem_we_r;
assign dmem_wdata = dmem_wdata_r;

integer errors;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dmem_req_r <= 1'b0;
        q_head     <= 0;
    end else begin
        if (dmem_req_r && dmem_ready) begin
            // Check a read's result the exact cycle completion is seen.
            if (q_is_read[q_head]) begin
                if (dmem_rdata !== q_expect[q_head]) begin
                    $display("[%0t] ERROR: read addr=%0d got %0d, expected %0d",
                              $time, q_addr[q_head], dmem_rdata, q_expect[q_head]);
                    errors = errors + 1;
                end else begin
                    $display("[%0t] OK: read addr=%0d = %0d", $time, q_addr[q_head], dmem_rdata);
                end
            end
            dmem_req_r <= 1'b0;
            q_head     <= q_head + 1;
        end else if (!dmem_req_r && (q_head < NREQ)) begin
            dmem_req_r    <= 1'b1;
            dmem_addr_r   <= q_addr[q_head];
            dmem_we_r     <= q_we[q_head];
            dmem_wdata_r  <= q_wdata[q_head];
        end
    end
end

initial begin
    rst_n = 1'b0;
    errors = 0;

    q_addr[0]=16'd5; q_we[0]=1'b1; q_wdata[0]=32'd100; q_is_read[0]=1'b0; q_expect[0]=32'd0;
    q_addr[1]=16'd6; q_we[1]=1'b1; q_wdata[1]=32'd200; q_is_read[1]=1'b0; q_expect[1]=32'd0;
    q_addr[2]=16'd5; q_we[2]=1'b0; q_wdata[2]=32'd0;   q_is_read[2]=1'b1; q_expect[2]=32'd100;
    q_addr[3]=16'd6; q_we[3]=1'b0; q_wdata[3]=32'd0;   q_is_read[3]=1'b1; q_expect[3]=32'd200;

    repeat (4) @(negedge clk);
    rst_n = 1'b1;

    wait (q_head == NREQ);
    repeat (10) @(negedge clk);

    $display("==============================================");
    if (aw_accepts !== 2) begin
        $display("[%0t] ERROR: expected exactly 2 AW accepts, saw %0d (spurious duplicate write!)", $time, aw_accepts);
        errors = errors + 1;
    end else begin
        $display("[%0t] OK: exactly 2 AW accepts, no duplicate writes", $time);
    end
    if (ar_accepts !== 2) begin
        $display("[%0t] ERROR: expected exactly 2 AR accepts, saw %0d (spurious duplicate read!)", $time, ar_accepts);
        errors = errors + 1;
    end else begin
        $display("[%0t] OK: exactly 2 AR accepts, no duplicate reads", $time);
    end

    if (errors == 0)
        $display("TB_CORE_AXI_ADAPTER: PASS (0 errors)");
    else
        $display("TB_CORE_AXI_ADAPTER: FAIL (%0d errors)", errors);
    $display("==============================================");
    $finish;
end

initial begin
    #100000;
    $display("[%0t] ERROR: global timeout (q_head stuck?)", $time);
    $display("TB_CORE_AXI_ADAPTER: FAIL (timeout)");
    $finish;
end

endmodule
