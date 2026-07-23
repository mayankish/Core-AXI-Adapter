// ============================================================
// Module      : core_axi_adapter
// Project     : core_axi_adapter - Tesla AI Hardware Portfolio
// Author      : Mayank
// Description : Converts riscv_ooo_core's dmem req/ready handshake
//               (dmem_addr/dmem_req/dmem_we/dmem_wdata/dmem_rdata/
//               dmem_ready - see riscv_ooo_core/rtl/lsu_frontend.v) into a
//               single-outstanding-transaction AXI4-Lite MASTER, so the
//               core's data-side memory accesses become bus-mastered
//               reads/writes reaching whatever a crossbar routes them to
//               (peripheral CSRs, a scratch RAM, etc).
//
//               This is why riscv_ooo_core's dmem interface grew the
//               dmem_req/dmem_ready handshake in the first place: the
//               original same-cycle interface (fine for a testbench's
//               behavioral RAM) can never be satisfied by a real AXI
//               transaction, which needs at least an address phase and a
//               data/response phase.
//
//               One request outstanding at a time (matches riscv_ooo_core's
//               own execution model - it never issues a second memory
//               request before the first completes, so there is nothing to
//               pipeline here). AW and W are issued together and accepted
//               independently (whichever of AWREADY/WREADY arrives first is
//               latched and held) - the same proven pattern used throughout
//               this portfolio's AXI4-Lite slaves (csr_block.v,
//               axi4s_dma.v, axi4l_slave_if.v), mirrored here from the
//               master side.
// ============================================================

module core_axi_adapter #(
    parameter AW = 16,   // AXI address width (matches the crossbar's master port)
    parameter DW = 32
)(
    input  wire            clk,
    input  wire            rst_n,

    // ---- riscv_ooo_core dmem port (slave side, from the core's POV) ----
    input  wire [AW-1:0]   dmem_addr,
    input  wire            dmem_req,
    input  wire            dmem_we,
    input  wire [DW-1:0]   dmem_wdata,
    output reg  [DW-1:0]   dmem_rdata,
    output wire             dmem_ready,   // combinational (Mealy) - see note below

    // ---- AXI4-Lite master (to the crossbar) ----
    output reg  [AW-1:0]   m_awaddr,
    output reg              m_awvalid,
    input  wire             m_awready,
    output reg  [DW-1:0]    m_wdata,
    output reg  [DW/8-1:0]  m_wstrb,
    output reg              m_wvalid,
    input  wire             m_wready,
    input  wire [1:0]       m_bresp,
    input  wire             m_bvalid,
    output reg              m_bready,
    output reg  [AW-1:0]    m_araddr,
    output reg              m_arvalid,
    input  wire             m_arready,
    input  wire [DW-1:0]    m_rdata,
    input  wire [1:0]       m_rresp,
    input  wire             m_rvalid,
    output reg              m_rready
);

localparam [2:0] ST_IDLE       = 3'd0,
                  ST_WRITE_ADDR = 3'd1,   // AW/W outstanding (independently)
                  ST_WRITE_RESP = 3'd2,   // waiting for B
                  ST_READ_ADDR  = 3'd3,   // AR outstanding
                  ST_READ_DATA  = 3'd4,   // waiting for R
                  ST_DONE       = 3'd5;   // 1-cycle dmem_ready pulse

reg [2:0] state;
reg       aw_done, w_done;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state      <= ST_IDLE;
        aw_done    <= 1'b0;
        w_done     <= 1'b0;
        m_awvalid  <= 1'b0;
        m_wvalid   <= 1'b0;
        m_bready   <= 1'b0;
        m_arvalid  <= 1'b0;
        m_rready   <= 1'b0;
        m_awaddr   <= {AW{1'b0}};
        m_wdata    <= {DW{1'b0}};
        m_wstrb    <= {(DW/8){1'b0}};
        m_araddr   <= {AW{1'b0}};
        dmem_rdata <= {DW{1'b0}};
    end else begin
        case (state)
            ST_IDLE: begin
                if (dmem_req && dmem_we) begin
                    m_awaddr  <= dmem_addr;
                    m_awvalid <= 1'b1;
                    m_wdata   <= dmem_wdata;
                    m_wstrb   <= {(DW/8){1'b1}};
                    m_wvalid  <= 1'b1;
                    aw_done   <= 1'b0;
                    w_done    <= 1'b0;
                    state     <= ST_WRITE_ADDR;
                end else if (dmem_req && !dmem_we) begin
                    m_araddr  <= dmem_addr;
                    m_arvalid <= 1'b1;
                    state     <= ST_READ_ADDR;
                end
            end

            ST_WRITE_ADDR: begin
                if (m_awvalid && m_awready) begin
                    m_awvalid <= 1'b0;
                    aw_done   <= 1'b1;
                end
                if (m_wvalid && m_wready) begin
                    m_wvalid <= 1'b0;
                    w_done   <= 1'b1;
                end
                // Both halves accepted (this cycle or a previous one) -> move on.
                if ((aw_done || (m_awvalid && m_awready)) &&
                    (w_done  || (m_wvalid  && m_wready))) begin
                    m_bready <= 1'b1;
                    state    <= ST_WRITE_RESP;
                end
            end

            ST_WRITE_RESP: begin
                if (m_bvalid && m_bready) begin
                    m_bready <= 1'b0;
                    state    <= ST_DONE;
                    // m_bresp is available here if error reporting is ever
                    // wired up - not consumed today (dmem has no error port).
                end
            end

            ST_READ_ADDR: begin
                if (m_arvalid && m_arready) begin
                    m_arvalid <= 1'b0;
                    m_rready  <= 1'b1;
                    state     <= ST_READ_DATA;
                end
            end

            ST_READ_DATA: begin
                if (m_rvalid && m_rready) begin
                    m_rready   <= 1'b0;
                    dmem_rdata <= m_rdata;
                    state      <= ST_DONE;
                end
            end

            ST_DONE: begin
                state <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

// Bug found via tb_core_axi_adapter.v (back-to-back write/write/read/read,
// zero idle cycles, req held as a real clocked register exactly like
// lsu_frontend's req_outstanding_r): dmem_ready was originally a
// REGISTERED pulse, asserted the cycle AFTER state left ST_DONE for
// ST_IDLE. That one-cycle lag meant ST_IDLE's "is this a fresh request"
// check could see dmem_req still asserted (the core hadn't yet observed
// dmem_ready and dropped its own request register) and start a SECOND,
// spurious transaction with stale address/data - confirmed empirically:
// 3 AW-channel accepts for 2 real writes, and subsequent reads came back
// shifted/wrong. Making dmem_ready a combinational (Mealy) output tied
// directly to `state == ST_DONE` removes the lag entirely: the adapter
// and the requester's own req-clearing logic now observe completion on
// the exact same cycle, so ST_IDLE can never see a stale request that
// dmem_ready itself just explained.
assign dmem_ready = (state == ST_DONE);

endmodule
