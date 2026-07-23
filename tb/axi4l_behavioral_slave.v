// ============================================================
// Module      : axi4l_behavioral_slave
// Project     : core_axi_adapter - Tesla AI Hardware Portfolio
// Author      : Mayank
// Description : Deliberately NOT same-cycle: introduces a couple of wait
//               states on each channel before asserting *ready/*valid, so
//               the adapter under test is actually exercised as a real
//               multi-cycle bus master, not accidentally validated only
//               against an idealized same-cycle responder. Small RAM
//               behind it so read-back can be checked directly.
// ============================================================

module axi4l_behavioral_slave #(
    parameter AW = 16,
    parameter DW = 32,
    parameter AW_DELAY = 2,   // cycles awvalid/arvalid must wait before *ready
    parameter B_DELAY  = 1,   // cycles after write-accept before bvalid
    parameter R_DELAY  = 2    // cycles after read-accept before rvalid
)(
    input  wire            clk,
    input  wire            rst_n,

    input  wire [AW-1:0]   s_awaddr,
    input  wire             s_awvalid,
    output reg              s_awready,
    input  wire [DW-1:0]    s_wdata,
    input  wire [DW/8-1:0]  s_wstrb,
    input  wire             s_wvalid,
    output reg              s_wready,
    output reg  [1:0]       s_bresp,
    output reg              s_bvalid,
    input  wire             s_bready,
    input  wire [AW-1:0]    s_araddr,
    input  wire             s_arvalid,
    output reg              s_arready,
    output reg  [DW-1:0]    s_rdata,
    output reg  [1:0]       s_rresp,
    output reg              s_rvalid,
    input  wire             s_rready
);

reg [DW-1:0] mem [0:(1<<8)-1];   // small backing store, word-addressed on [7:0]

integer aw_cnt, ar_cnt;
reg aw_latched, ar_latched;
reg [AW-1:0] aw_addr_r, ar_addr_r;
reg w_latched;
reg [DW-1:0] w_data_r;

reg b_pending;
integer b_cnt;
reg r_pending;
integer r_cnt;

// ---- AW/W accept after AW_DELAY cycles of both being valid ------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        s_awready <= 1'b0; s_wready <= 1'b0;
        aw_cnt <= 0; aw_latched <= 1'b0; w_latched <= 1'b0;
    end else begin
        s_awready <= 1'b0;
        s_wready  <= 1'b0;
        if (!aw_latched) begin
            if (s_awvalid) begin
                if (aw_cnt >= AW_DELAY) begin
                    s_awready  <= 1'b1;
                    aw_addr_r  <= s_awaddr;
                    aw_latched <= 1'b1;
                    aw_cnt     <= 0;
                end else aw_cnt <= aw_cnt + 1;
            end else aw_cnt <= 0;
        end
        if (!w_latched) begin
            if (s_wvalid) begin
                // piggy-back on the same delay counter for simplicity
                if (aw_cnt >= AW_DELAY || aw_latched) begin
                    s_wready  <= 1'b1;
                    w_data_r  <= s_wdata;
                    w_latched <= 1'b1;
                end
            end
        end
        if (aw_latched && w_latched) begin
            aw_latched <= 1'b0;
            w_latched  <= 1'b0;
        end
    end
end

// ---- B response B_DELAY cycles after both AW and W landed --------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        s_bvalid <= 1'b0; s_bresp <= 2'b00; b_pending <= 1'b0; b_cnt <= 0;
    end else begin
        if (s_awready && s_wready) begin
            mem[aw_addr_r[7:0]] <= s_wdata;
            b_pending <= 1'b1;
            b_cnt <= 0;
        end else if (b_pending && !s_bvalid) begin
            if (b_cnt >= B_DELAY) begin
                s_bvalid <= 1'b1;
                s_bresp  <= 2'b00;
            end else b_cnt <= b_cnt + 1;
        end
        if (s_bvalid && s_bready) begin
            s_bvalid  <= 1'b0;
            b_pending <= 1'b0;
        end
    end
end

// ---- AR accept after AW_DELAY cycles -----------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        s_arready <= 1'b0; ar_cnt <= 0;
    end else begin
        s_arready <= 1'b0;
        if (s_arvalid && !r_pending) begin
            if (ar_cnt >= AW_DELAY) begin
                s_arready <= 1'b1;
                ar_addr_r <= s_araddr;
                ar_cnt    <= 0;
            end else ar_cnt <= ar_cnt + 1;
        end else if (!s_arvalid) ar_cnt <= 0;
    end
end

// ---- R response R_DELAY cycles after AR accepted -----------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        s_rvalid <= 1'b0; s_rresp <= 2'b00; r_pending <= 1'b0; r_cnt <= 0; s_rdata <= {DW{1'b0}};
    end else begin
        if (s_arready) begin
            r_pending <= 1'b1;
            r_cnt <= 0;
        end else if (r_pending && !s_rvalid) begin
            if (r_cnt >= R_DELAY) begin
                s_rvalid <= 1'b1;
                s_rdata  <= mem[ar_addr_r[7:0]];
                s_rresp  <= 2'b00;
            end else r_cnt <= r_cnt + 1;
        end
        if (s_rvalid && s_rready) begin
            s_rvalid  <= 1'b0;
            r_pending <= 1'b0;
        end
    end
end

endmodule
