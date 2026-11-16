// =============================================================================
// Project:     HUST ET4030 - ASIC Design Project
// Module Name: apb_uart
// Description: Top-Level APB-UART Core IP integrating APB4 Slave Register File,
//              Fractional Baud Rate Generator, UART Transmitter, UART Receiver,
//              and Peripheral Handshake Synchronizers.
// Standard:    AMBA APB4 (Arm IHI 0024C), Spec v1.1 / v2.0 Architecture Frozen
// Target:      Tiny Tapeout Sky130A (f_pclk = 25 MHz, T = 40 ns)
// =============================================================================
// Features:
//   - Zero Wait-State AMBA APB4 Slave interface (pready = 1 fixed)
//   - Strict address decoding & E1-E5 APB protocol error checking
//   - 7 Memory-Mapped 32-bit registers (TXDATA, RXDATA, CFG, CTRL, STAT, BAUD, IER)
//   - 24 UART frame configurations (5-8 data bits, Parity Even/Odd/None, 1-2 stop bits)
//   - 16x fractional accumulator baud generator (2,400 to 921,600 bps)
//   - Hardware flow control: cts_n input (with 2FF sync) & rts_n output
//   - Glitch-filtered RX center sampling (tick 8) with +-2% baud tolerance
//   - Clean registered level interrupt (irq) and 2-cycle hardware soft reset
//   - Single clock domain (pclk) with active-low asynchronous reset (presetn)
// =============================================================================

module apb_uart (
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
    output wire        irq          // Registered level-high interrupt output
);
    // Preserve the specified APB-only interface while sharing one RTL core
    // with the Tiny Tapeout wrapper. Status outputs are unused in this facade.
    wire [2:0] unused_status;
    apb_uart_core u_core (
        .pclk(pclk), .presetn(presetn),
        .psel(psel), .penable(penable), .pwrite(pwrite),
        .paddr(paddr), .pstrb(pstrb), .pwdata(pwdata),
        .pready(pready), .pslverr(pslverr), .prdata(prdata),
        .rx(rx), .cts_n(cts_n), .tx(tx), .rts_n(rts_n), .irq(irq),
        .tx_done(unused_status[0]), .rx_done(unused_status[1]),
        .parity_err(unused_status[2])
    );
endmodule
