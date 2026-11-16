// Shared APB-UART implementation, including observable TT status outputs.
// apb_uart is the fixed-interface facade; the TT wrapper instantiates this core.
`default_nettype none

module apb_uart_core (
    // -------------------------------------------------------------------------
    // Clock and Reset Interface
    // -------------------------------------------------------------------------
    input  wire        pclk,        // System clock (nominal 25 MHz)
    input  wire        presetn,     // Asynchronous active-low system reset

    // -------------------------------------------------------------------------
    // AMBA APB4 Slave Interface
    // -------------------------------------------------------------------------
    input  wire        psel,        // APB select
    input  wire        penable,     // APB enable (access phase)
    input  wire        pwrite,      // APB write (1 = write, 0 = read)
    input  wire [11:0] paddr,       // APB byte address [11:0]
    input  wire [3:0]  pstrb,       // APB write byte lane strobes
    input  wire [31:0] pwdata,      // APB write data
    output wire        pready,      // APB slave ready (fixed 1'b1)
    output wire        pslverr,     // APB slave error response (E1-E5)
    output wire [31:0] prdata,      // APB read data

    // -------------------------------------------------------------------------
    // UART Physical Serial & Flow Control Interface
    // -------------------------------------------------------------------------
    input  wire        rx,          // Asynchronous serial RX input
    input  wire        cts_n,       // Asynchronous Clear-to-Send input (active-low)
    output wire        tx,          // Serial TX output (idle = 1'b1)
    output wire        rts_n,       // Request-to-Send output (active-low)

    // -------------------------------------------------------------------------
    // System Level Interrupt Output
    // -------------------------------------------------------------------------
    output wire        irq,         // Registered level-high interrupt output
    output wire        tx_done,     // Transmitter idle/done level
    output wire        rx_done,     // RXDATA contains an unread character
    output wire        parity_err   // Stored RXDATA parity-error flag
);

    // =========================================================================
    // 1. ASYNCHRONOUS PERIPHERAL INPUT SYNCHRONIZATION (Q6 COMPLIANT)
    // =========================================================================
    // cts_n is synchronized into the pclk domain via a 2-stage Flip-Flop synchronizer.
    // Note: rx is synchronized internally within uart_rx via its own 2FF synchronizer.
    (* async_reg = "true" *) reg cts_sync1;
    (* async_reg = "true" *) reg cts_sync2;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            cts_sync1 <= 1'b1;
            cts_sync2 <= 1'b1;
        end else begin
            cts_sync1 <= cts_n;
            cts_sync2 <= cts_sync1;
        end
    end

    wire cts_s = cts_sync2;
    wire unused_frame_err, unused_overrun, unused_rx_rts;

    // =========================================================================
    // 2. INTERNAL CONTROL & RESET DISTRIBUTION
    // =========================================================================
    // soft_rst is asserted by apb_regs for exactly 2 pclk cycles when CTRL[8]=1.
    // internal_rst_n combines presetn and ~soft_rst to drive the active-low asynchronous reset
    // inputs of sub-blocks (baud_gen, uart_tx, uart_rx), aborting active frames, returning FSMs
    // to IDLE, and resetting accumulators.
    wire soft_rst;
    wire internal_rst_n = presetn & ~soft_rst;

    // =========================================================================
    // 3. INTERNAL INTERCONNECT WIRES
    // =========================================================================
    // Baud Generator wires
    wire [11:0] div_int;
    wire [3:0]  div_frac;
    wire        tick;

    // Frame Configuration & Control wires
    wire [1:0]  data_bits;
    wire        stop_bits;
    wire        parity_en;
    wire        parity_type;
    wire        tx_dis;
    wire        rx_dis;

    // Transmitter wires
    wire [7:0]  tx_data;
    wire        start_tx;
    // Receiver wires
    wire [7:0]  rx_data;
    wire        rx_valid;
    wire        rx_busy;
    wire        rx_parity_err;
    wire        rx_frame_err;
    // =========================================================================
    // 4. MODULE INSTANTIATIONS
    // =========================================================================

    // -------------------------------------------------------------------------
    // Submodule 1: apb_regs (AMBA APB4 Slave Register File)
    // -------------------------------------------------------------------------
    apb_regs u_apb_regs (
        .pclk          (pclk),
        .presetn       (presetn),          // Dedicated hardware reset

        // APB4 Slave Bus
        .psel          (psel),
        .penable       (penable),
        .pwrite        (pwrite),
        .paddr         (paddr),
        .pstrb         (pstrb),
        .pwdata        (pwdata),
        .pready        (pready),
        .pslverr       (pslverr),
        .prdata        (prdata),

        // Transmitter Interface
        .tx_data       (tx_data),
        .start_tx      (start_tx),
        .tx_done       (tx_done),

        // Receiver Interface
        .rx_data_in    (rx_data),
        .rx_valid      (rx_valid),
        .rx_busy       (rx_busy),
        .parity_err_in (rx_parity_err),
        .frame_err_in  (rx_frame_err),

        // Frame Configuration & Control
        .data_bits     (data_bits),
        .stop_bits     (stop_bits),
        .parity_en     (parity_en),
        .parity_type   (parity_type),
        .tx_dis        (tx_dis),
        .rx_dis        (rx_dis),

        // Baud Rate Divisor Output
        .div_int       (div_int),
        .div_frac      (div_frac),

        // Status & Interrupt
        .rx_done       (rx_done),
        .parity_err    (parity_err),       // Also exported to TT status pin
        .frame_err     (unused_frame_err),                 // Read through STAT register
        .overrun       (unused_overrun),                 // Read through STAT register
        .rts_n         (rts_n),
        .irq           (irq),
        .soft_rst      (soft_rst)
    );

    // -------------------------------------------------------------------------
    // Submodule 2: baud_gen (Fractional Baud Rate Generator)
    // -------------------------------------------------------------------------
    baud_gen u_baud_gen (
        .pclk          (pclk),
        .presetn       (internal_rst_n),   // Hardware reset + Soft reset
        .div_int       (div_int),
        .div_frac      (div_frac),
        .tick          (tick)
    );

    // -------------------------------------------------------------------------
    // Submodule 3: uart_tx (UART Serial Transmitter)
    // -------------------------------------------------------------------------
    uart_tx u_uart_tx (
        .pclk          (pclk),
        .presetn       (internal_rst_n),   // Hardware reset + Soft reset
        .tick          (tick),
        .tx_data       (tx_data),
        .start         (start_tx),
        .cts_s         (cts_s),            // Synchronized CTS input
        .tx_dis        (tx_dis),
        .data_bits     (data_bits),
        .parity_en     (parity_en),
        .parity_type   (parity_type),
        .stop_bits     (stop_bits),
        .tx            (tx),
        .tx_done       (tx_done)
    );

    // -------------------------------------------------------------------------
    // Submodule 4: uart_rx (UART Serial Receiver)
    // -------------------------------------------------------------------------
    uart_rx u_uart_rx (
        .pclk          (pclk),
        .presetn       (internal_rst_n),   // Hardware reset + Soft reset
        .tick          (tick),
        .rx            (rx),               // Synchronized internally (2FF)
        .rx_dis        (rx_dis),
        .rx_done       (rx_done),
        .data_bits     (data_bits),
        .parity_en     (parity_en),
        .parity_type   (parity_type),
        .stop_bits     (stop_bits),
        .rx_data       (rx_data),
        .rx_valid      (rx_valid),
        .rx_busy       (rx_busy),
        .parity_err    (rx_parity_err),
        .frame_err     (rx_frame_err),
        .rts_n         (unused_rx_rts)                  // Handled cleanly via apb_regs.rts_n
    );

endmodule

`default_nettype wire
