// =============================================================================
// Module Name: apb_regs
// Description: AMBA APB4 Slave Register File for UART core
// Standard:    AMBA APB4 (Arm IHI 0024C), Spec v2.0 Architecture Frozen
// Target:      Tiny Tapeout Sky130A (f_pclk = 25 MHz, T = 40 ns)
// Features:
//   - Zero Wait-State (pready = 1'b1 fixed)
//   - Strict address decoding (7 aligned registers, unaligned / out-of-range = E1)
//   - APB error detection E1-E5 with pstrb support
//   - tx_launch_pending guard to eliminate TX race conditions
//   - Priority-based RX race condition handling (CPU Read vs Hardware rx_valid)
//   - Registered 1-stage Flip-Flop IRQ output (glitch-free, delay = 1 pclk)
//   - 2-cycle hardware soft reset controller (CTRL.soft_rst)
// =============================================================================

module apb_regs (
    input  wire        pclk,
    input  wire        presetn,

    // -------------------------------------------------------------------------
    // AMBA APB4 Slave Bus Interface
    // -------------------------------------------------------------------------
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [11:0] paddr,
    input  wire [3:0]  pstrb,
    input  wire [31:0] pwdata,
    output wire        pready,
    output wire        pslverr,
    output reg  [31:0] prdata,

    // -------------------------------------------------------------------------
    // Internal Interface to uart_tx
    // -------------------------------------------------------------------------
    output wire [7:0]  tx_data,
    output reg         start_tx,
    input  wire        tx_done,

    // -------------------------------------------------------------------------
    // Internal Interface to uart_rx
    // -------------------------------------------------------------------------
    input  wire [7:0]  rx_data_in,
    input  wire        rx_valid,
    input  wire        rx_busy,
    input  wire        parity_err_in,
    input  wire        frame_err_in,

    // -------------------------------------------------------------------------
    // Frame Configuration & Control (to uart_tx, uart_rx)
    // -------------------------------------------------------------------------
    output wire [1:0]  data_bits,
    output wire        stop_bits,
    output wire        parity_en,
    output wire        parity_type,
    output wire        tx_dis,
    output wire        rx_dis,

    // -------------------------------------------------------------------------
    // Internal Interface to baud_gen
    // -------------------------------------------------------------------------
    output wire [11:0] div_int,
    output wire [3:0]  div_frac,

    // -------------------------------------------------------------------------
    // Status Flags, Peripheral Handshake & Interrupt
    // -------------------------------------------------------------------------
    output reg         rx_done,
    output reg         parity_err,
    output reg         frame_err,
    output reg         overrun,
    output wire        rts_n,
    output reg         irq,
    output wire        soft_rst
);

    // =========================================================================
    // 1. REGISTER ADDRESS MAP & INTERNAL REGISTERS
    // =========================================================================
    localparam [11:0]
        ADDR_TXDATA = 12'h000,
        ADDR_RXDATA = 12'h004,
        ADDR_CFG    = 12'h008,
        ADDR_CTRL   = 12'h00C,
        ADDR_STAT   = 12'h010,
        ADDR_BAUD   = 12'h014,
        ADDR_IER    = 12'h018;

    // Internal storage registers
    reg [7:0]  r_txdata;
    reg [7:0]  r_rxdata;
    reg [6:0]  r_cfg;   // [1:0] data_bits, [2] stop_bits, [3] parity_en, [4] parity_type, [5] tx_dis, [6] rx_dis
    reg [15:0] r_baud;  // [11:0] div_int, [15:12] div_frac
    reg [2:0]  r_ier;   // [0] tx_done_ie, [1] rx_done_ie, [2] err_ie

    // Control & Protection registers
    reg        tx_launch_pending;
    reg [1:0]  soft_rst_cnt; // Shift sequence 11 -> 01 -> 00, two active cycles.

    // =========================================================================
    // 2. CONTINUOUS OUTPUT ASSIGNMENTS & STATUS SIGNALS
    // =========================================================================
    assign pready      = 1'b1; // Zero wait-state

    assign tx_data     = r_txdata;
    assign data_bits   = r_cfg[1:0];
    assign stop_bits   = r_cfg[2];
    assign parity_en   = r_cfg[3];
    assign parity_type = r_cfg[4];
    assign tx_dis      = r_cfg[5];
    assign rx_dis      = r_cfg[6];

    assign div_int     = r_baud[11:0];
    assign div_frac    = r_baud[15:12];

    // Soft reset active state
    wire soft_rst_active = (soft_rst_cnt != 2'd0);
    assign soft_rst      = soft_rst_active;

    // Hardware flow control RTS (Active-low: 0 = Ready, 1 = Not ready / Busy)
    // FIX TOP-01: During hard reset (!presetn) or active soft reset (soft_rst_active),
    // receiver hardware is not ready to accept bytes, so rts_n must be held high (1'b1).
    // Once reset releases, if rx_dis=0 and rx_done=0, rts_n transitions to 0 (Ready).
    assign rts_n       = (!presetn) || soft_rst_active || r_cfg[6] || rx_done;

    // Transmitter status conditions (Fix A1: E3 only depends on tx_busy)
    wire tx_busy          = !tx_done || tx_launch_pending;
    wire tx_start_allowed = !tx_busy && !r_cfg[5];

    // =========================================================================
    // 3. APB BUS ACCESS DECODE & ERROR CHECKING (E1 - E5)
    // =========================================================================
    wire apb_access = presetn && psel && penable;

    // Exact aligned 4-byte address check
    wire valid_aligned_addr = (
        (paddr == ADDR_TXDATA) || (paddr == ADDR_RXDATA) ||
        (paddr == ADDR_CFG)    || (paddr == ADDR_CTRL)   ||
        (paddr == ADDR_STAT)   || (paddr == ADDR_BAUD)   ||
        (paddr == ADDR_IER)
    );

    // Identify soft reset request from APB write
    wire soft_rst_req = apb_access && pwrite && (paddr == ADDR_CTRL) && pstrb[1] && pwdata[8];

    // Calculate new div_int after applying byte strobe (Spec v2.0 Section 4.6)
    wire [11:0] new_div_int = {
        pstrb[1] ? pwdata[11:8] : r_baud[11:8],
        pstrb[0] ? pwdata[7:0]  : r_baud[7:0]
    };

    // E1: Invalid or unaligned address
    wire err_e1 = !valid_aligned_addr;

    // E2: Write to Read-Only register (RXDATA or STAT, regardless of pstrb)
    wire err_e2 = pwrite && valid_aligned_addr && ((paddr == ADDR_RXDATA) || (paddr == ADDR_STAT));

    // E3: Write TXDATA or CTRL.start_tx when TX is busy/pending (Fix A1: independent of tx_dis)
    wire err_e3 = pwrite && valid_aligned_addr && !soft_rst_req && (
        ((paddr == ADDR_TXDATA) && (|pstrb) && tx_busy) ||
        ((paddr == ADDR_CTRL)   && pstrb[0] && pwdata[0] && tx_busy)
    );

    // E4: Dynamic reconfiguration of CFG or BAUD when TX or RX is busy
    wire err_e4 = pwrite && valid_aligned_addr && (!tx_done || tx_launch_pending || rx_busy) && (
        ((paddr == ADDR_CFG)  && pstrb[0]) ||
        ((paddr == ADDR_BAUD) && (|pstrb[1:0]))
    );

    // E5: Write to BAUD resulting in div_int == 0
    wire err_e5 = pwrite && valid_aligned_addr && (paddr == ADDR_BAUD) && (|pstrb[1:0]) && (new_div_int == 12'd0);

    // Final pslverr response (active only in ACCESS phase, never during soft reset)
    assign pslverr = apb_access && !soft_rst_req && !soft_rst_active &&
                     (err_e1 || err_e2 || err_e3 || err_e4 || err_e5);

    // =========================================================================
    // 4. READ DATA MULTIPLEXER (prdata)
    // =========================================================================
    always @(*) begin
        if (apb_access && !pwrite && valid_aligned_addr) begin
            case (paddr)
                ADDR_TXDATA: prdata = {24'd0, r_txdata};
                ADDR_RXDATA: prdata = {24'd0, r_rxdata};
                ADDR_CFG:    prdata = {25'd0, r_cfg};
                ADDR_CTRL:   prdata = 32'd0; // Command bits read as 0
                ADDR_STAT:   prdata = {24'd0, overrun, frame_err, r_cfg[6], r_cfg[5], rx_busy, parity_err, rx_done, tx_done};
                ADDR_BAUD:   prdata = {16'd0, r_baud};
                ADDR_IER:    prdata = {29'd0, r_ier};
                default:     prdata = 32'd0;
            endcase
        end else begin
            prdata = 32'd0; // Default zero outside ACCESS or on error
        end
    end

    // =========================================================================
    // 5. SEQUENTIAL LOGIC & STATE UPDATES (posedge pclk)
    // =========================================================================
    wire rxdata_read_fire = apb_access && !pwrite && (paddr == ADDR_RXDATA) && !pslverr && !soft_rst_active;

    // Interrupt condition combinatorial source
    wire irq_comb = (r_ier[0] & tx_done) |
                    (r_ier[1] & rx_done) |
                    (r_ier[2] & (parity_err | frame_err | overrun));

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            // Asynchronous Hard Reset (Default Power-on values)
            r_txdata          <= 8'h00;
            r_rxdata          <= 8'h00;
            r_cfg             <= 7'h00;
            r_baud            <= 16'h900D; // 115200 baud at 25 MHz (D = 13.5625)
            r_ier             <= 3'b000;
            rx_done           <= 1'b0;
            parity_err        <= 1'b0;
            frame_err         <= 1'b0;
            overrun           <= 1'b0;
            start_tx          <= 1'b0;
            tx_launch_pending <= 1'b0;
            soft_rst_cnt      <= 2'd0;
            irq               <= 1'b0;
        end else if (soft_rst_active) begin
            // -----------------------------------------------------------------
            // Hardware Soft Reset in Progress (2 pclk cycles)
            // -----------------------------------------------------------------
            soft_rst_cnt      <= {1'b0, soft_rst_cnt[1]};
            r_txdata          <= 8'h00;
            r_rxdata          <= 8'h00;
            r_cfg             <= 7'h00;
            r_baud            <= 16'h900D;
            r_ier             <= 3'b000;
            rx_done           <= 1'b0;
            parity_err        <= 1'b0;
            frame_err         <= 1'b0;
            overrun           <= 1'b0;
            start_tx          <= 1'b0;
            tx_launch_pending <= 1'b0;
            irq               <= 1'b0;
        end else begin
            // -----------------------------------------------------------------
            // Normal Operating Mode
            // -----------------------------------------------------------------
            // Default 1-cycle auto-clearing command pulse
            start_tx <= 1'b0;

            // Clear launch pending once uart_tx reflects busy state (tx_done == 0)
            if (!tx_done) begin
                tx_launch_pending <= 1'b0;
            end

            // -----------------------------------------------------------------
            // RX Events & Read Side-Effect Handling (Spec v2.0 Section 7)
            // Priority Rule: Clear-then-Load on same-cycle read & rx_valid
            // -----------------------------------------------------------------
            if (rxdata_read_fire && rx_valid) begin
                // CPU read succeeds on old data; new incoming byte is loaded simultaneously
                r_rxdata   <= rx_data_in;
                rx_done    <= 1'b1;
                parity_err <= parity_err_in;
                frame_err  <= frame_err_in;
                overrun    <= 1'b0; // Not overrun because old byte was consumed
            end else if (rxdata_read_fire) begin
                // Normal read: clear all 4 RX status flags
                rx_done    <= 1'b0;
                parity_err <= 1'b0;
                frame_err  <= 1'b0;
                overrun    <= 1'b0;
            end else if (rx_valid) begin
                // Hardware incoming byte arrived without CPU read
                if (rx_done) begin
                    overrun <= 1'b1; // Buffer full: keep old byte, flag overrun
                end else begin
                    r_rxdata   <= rx_data_in;
                    rx_done    <= 1'b1;
                    parity_err <= parity_err_in;
                    frame_err  <= frame_err_in;
                end
            end

            // -----------------------------------------------------------------
            // APB Write Transactions Commit
            // -----------------------------------------------------------------
            if (soft_rst_req) begin
                // Soft reset command triggered: Load 2-cycle timer & reset registers
                soft_rst_cnt      <= 2'b11;
                r_txdata          <= 8'h00;
                r_rxdata          <= 8'h00;
                r_cfg             <= 7'h00;
                r_baud            <= 16'h900D;
                r_ier             <= 3'b000;
                rx_done           <= 1'b0;
                parity_err        <= 1'b0;
                frame_err         <= 1'b0;
                overrun           <= 1'b0;
                start_tx          <= 1'b0;
                tx_launch_pending <= 1'b0;
                irq               <= 1'b0;
            end else if (apb_access && pwrite && !pslverr) begin
                case (paddr)
                    ADDR_TXDATA: begin
                        if (pstrb[0]) r_txdata <= pwdata[7:0];
                    end

                    ADDR_CFG: begin
                        if (pstrb[0]) r_cfg <= pwdata[6:0];
                    end

                    ADDR_CTRL: begin
                        // Launch transmission if start requested and transmitter start allowed
                        if (pstrb[0] && pwdata[0] && tx_start_allowed) begin
                            start_tx          <= 1'b1;
                            tx_launch_pending <= 1'b1;
                        end
                    end

                    ADDR_BAUD: begin
                        if (pstrb[0]) r_baud[7:0]  <= pwdata[7:0];
                        if (pstrb[1]) r_baud[15:8] <= pwdata[15:8];
                    end

                    ADDR_IER: begin
                        if (pstrb[0]) r_ier <= pwdata[2:0];
                    end
                    default: begin end // RO/invalid addresses cannot commit.
                endcase
            end

            // -----------------------------------------------------------------
            // Registered Interrupt Generation (1-stage DFF for clean output)
            // Fix A3: soft_rst_req takes immediate precedence to clear IRQ
            // -----------------------------------------------------------------
            if (soft_rst_req) begin
                irq <= 1'b0;
            end else begin
                irq <= irq_comb;
            end
        end
    end

// Upper write-data bits are reserved and intentionally ignored.
    wire unused_pwdata = &{1'b0, pwdata[31:16]};
endmodule
