// =============================================================================
// Module: uart_rx
// Description: UART Receiver with 2FF synchronizer, 16x oversampling, center
//              sampling at tick 8, glitch rejection (< 0.4 bit), index-based
//              SIPO for 5-8 bit data (upper bits 0), Even/Odd parity checker,
//              framing error detection (checking all stop bits), early return
//              to IDLE at tick 8 of the last stop bit (for +-2% baud tolerance),
//              and 1-pclk-pulse rx_valid.
// Reference: uart_tx_rx_spec_v2.md, HD-APB-UART-tuong-tac.html
// Standard: Verilog-2001, synchronous to pclk, active-low presetn
// =============================================================================

module uart_rx (
    input  wire       pclk,        // System clock (25 MHz)
    input  wire       presetn,     // Asynchronous active-low reset
    input  wire       tick,        // Baud tick enable from baud_gen (1 pclk wide)
    input  wire       rx,          // Asynchronous serial input line
    input  wire       rx_dis,      // 1: Disable RX
    input  wire       rx_done,     // From apb_regs: 1 if RXDATA unread (for rts_n)
    input  wire [1:0] data_bits,   // 00: 5-bit, 01: 6-bit, 10: 7-bit, 11: 8-bit
    input  wire       parity_en,   // 1: Enable parity
    input  wire       parity_type, // 0: Odd, 1: Even (spec v2.0 & HTML line 442)
    input  wire       stop_bits,   // 0: 1 stop bit, 1: 2 stop bits
    output reg  [7:0] rx_data,     // Received data byte (upper bits 0 for N < 8)
    output reg        rx_valid,    // 1-cycle pulse on completed valid/invalid frame
    output reg        rx_busy,     // 1 while actively receiving a frame
    output reg        parity_err,  // 1 if parity mismatch detected
    output reg        frame_err,   // 1 if any stop bit sampled as 0
    output wire       rts_n        // Hardware flow control: rx_dis | rx_done
);

    // -------------------------------------------------------------------------
    // Hardware Flow Control (Spec v2.0 Section 5.9, HTML line 391)
    // -------------------------------------------------------------------------
    assign rts_n = rx_dis | rx_done;

    // -------------------------------------------------------------------------
    // 2-Stage Flip-Flop Synchronizer for Asynchronous RX Input (Spec Section 5.3)
    // -------------------------------------------------------------------------
    (* async_reg = "true" *) reg rx_sync1;
    (* async_reg = "true" *) reg rx_sync2;
    reg rx_sync2_d;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            rx_sync1   <= 1'b1;
            rx_sync2   <= 1'b1;
            rx_sync2_d <= 1'b1;
        end else begin
            rx_sync1   <= rx;
            rx_sync2   <= rx_sync1;
            rx_sync2_d <= rx_sync2;
        end
    end

    // Falling edge detection on synchronized RX line
    wire rx_falling_edge = (rx_sync2_d == 1'b1) && (rx_sync2 == 1'b0);

    // -------------------------------------------------------------------------
    // FSM State Encoding
    // -------------------------------------------------------------------------
    localparam [2:0]
        RX_IDLE   = 3'd0,
        RX_START  = 3'd1,
        RX_DATA   = 3'd2,
        RX_PARITY = 3'd3,
        RX_STOP   = 3'd4;

    reg [2:0] state;

    // -------------------------------------------------------------------------
    // Internal Datapath Registers
    // -------------------------------------------------------------------------
    reg [3:0] tick_cnt;         // Sub-bit tick counter (0..15)
    reg [3:0] bit_idx;          // Data bit index / stop bit index
    reg [7:0] rx_shift_reg;     // SIPO buffer

    // Frame configuration registers
    reg [3:0] frame_len_q;      // 5, 6, 7, 8
    reg       frame_par_en_q;
    reg       frame_par_type_q;
    reg       frame_stop2_q;

    // Data mask to clear unused upper bits for N < 8
    wire [7:0] data_mask = (frame_len_q == 4'd5) ? 8'h1F :
                           (frame_len_q == 4'd6) ? 8'h3F :
                           (frame_len_q == 4'd7) ? 8'h7F : 8'hFF;

    // Parity calculation on received data
    wire [7:0] masked_rx_data = rx_shift_reg & data_mask;
    wire       p_calc_even    = ^masked_rx_data;
    wire       p_calc_odd     = ~p_calc_even;
    wire       p_expected     = frame_par_type_q ? p_calc_even : p_calc_odd;

    // -------------------------------------------------------------------------
    // FSM & Datapath Sequential Logic
    // -------------------------------------------------------------------------
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            state            <= RX_IDLE;
            rx_data          <= 8'h00;
            rx_valid         <= 1'b0;
            rx_busy          <= 1'b0;
            parity_err       <= 1'b0;
            frame_err        <= 1'b0;
            tick_cnt         <= 4'd0;
            bit_idx          <= 4'd0;
            rx_shift_reg     <= 8'h00;
            frame_len_q      <= 4'd8;
            frame_par_en_q   <= 1'b0;
            frame_par_type_q <= 1'b0;
            frame_stop2_q    <= 1'b0;
        end else begin
            // Default 1-pclk-pulse behavior: rx_valid is high for exactly 1 cycle
            rx_valid <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // RX_IDLE: Awaiting Start bit falling edge
                // -------------------------------------------------------------
                RX_IDLE: begin
                    rx_busy  <= 1'b0;
                    tick_cnt <= 4'd0;
                    bit_idx  <= 4'd0;

                    if (rx_falling_edge && !rx_dis) begin
                        state            <= RX_START;
                        rx_busy          <= 1'b1;
                        tick_cnt         <= 4'd0;
                        bit_idx          <= 4'd0;
                        rx_shift_reg     <= 8'h00;
                        parity_err       <= 1'b0;
                        frame_err        <= 1'b0;
                        // Capture frame configuration at start of frame
                        frame_len_q      <= 4'd5 + {2'b00, data_bits};
                        frame_par_en_q   <= parity_en;
                        frame_par_type_q <= parity_type;
                        frame_stop2_q    <= stop_bits;
                    end
                end

                // -------------------------------------------------------------
                // RX_START: Verify Start bit at mid-bit (tick 8) to reject glitches
                // -------------------------------------------------------------
                RX_START: begin
                    rx_busy <= 1'b1;
                    if (tick) begin
                        if (tick_cnt == 4'd8) begin
                            if (rx_sync2 == 1'b1) begin
                                // Glitch detected (< 0.4 bit): abort and return to IDLE
                                state    <= RX_IDLE;
                                rx_busy  <= 1'b0;
                                tick_cnt <= 4'd0;
                            end else begin
                                tick_cnt <= tick_cnt + 1'b1;
                            end
                        end else if (tick_cnt == 4'd15) begin
                            // Start bit fully elapsed (16 ticks); advance to DATA
                            tick_cnt <= 4'd0;
                            bit_idx  <= 4'd0;
                            state    <= RX_DATA;
                        end else begin
                            tick_cnt <= tick_cnt + 1'b1;
                        end
                    end
                end

                // -------------------------------------------------------------
                // RX_DATA: Sample N data bits at tick 8 (index-based SIPO)
                // -------------------------------------------------------------
                RX_DATA: begin
                    rx_busy <= 1'b1;
                    if (tick) begin
                        if (tick_cnt == 4'd8) begin
                            // Sample at middle of bit and store directly by index
                            rx_shift_reg[bit_idx[2:0]] <= rx_sync2;
                            tick_cnt              <= tick_cnt + 1'b1;
                        end else if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;
                            if (bit_idx < frame_len_q - 1'b1) begin
                                bit_idx <= bit_idx + 1'b1;
                            end else begin
                                // All N data bits sampled
                                bit_idx <= 4'd0;
                                if (frame_par_en_q) begin
                                    state <= RX_PARITY;
                                end else begin
                                    state <= RX_STOP;
                                end
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 1'b1;
                        end
                    end
                end

                // -------------------------------------------------------------
                // RX_PARITY: Sample and verify Parity bit at tick 8
                // -------------------------------------------------------------
                RX_PARITY: begin
                    rx_busy <= 1'b1;
                    if (tick) begin
                        if (tick_cnt == 4'd8) begin
                            // Check parity mismatch
                            if (rx_sync2 != p_expected) begin
                                parity_err <= 1'b1;
                            end
                            tick_cnt <= tick_cnt + 1'b1;
                        end else if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;
                            bit_idx  <= 4'd0;
                            state    <= RX_STOP;
                        end else begin
                            tick_cnt <= tick_cnt + 1'b1;
                        end
                    end
                end

                // -------------------------------------------------------------
                // RX_STOP: Sample Stop bit(s) at tick 8; early return on last stop
                // -------------------------------------------------------------
                RX_STOP: begin
                    rx_busy <= 1'b1;
                    if (tick) begin
                        if (!frame_stop2_q) begin
                            // -------------------------------------------------
                            // 1 Stop bit mode: sample at tick 8 and return IDLE
                            // -------------------------------------------------
                            if (tick_cnt == 4'd8) begin
                                if (rx_sync2 == 1'b0) begin
                                    frame_err <= 1'b1;
                                end
                                rx_data  <= rx_shift_reg & data_mask;
                                rx_valid <= 1'b1; // 1-cycle pulse
                                // Early return to IDLE at tick 8 (Spec v2.0 Section 5.7)
                                state    <= RX_IDLE;
                                rx_busy  <= 1'b0;
                                tick_cnt <= 4'd0;
                                bit_idx  <= 4'd0;
                            end else begin
                                tick_cnt <= tick_cnt + 1'b1;
                            end
                        end else begin
                            // -------------------------------------------------
                            // 2 Stop bits mode: check both stop bits
                            // -------------------------------------------------
                            if (bit_idx == 4'd0) begin
                                // Stop bit 1: sample at tick 8, full 16 ticks
                                if (tick_cnt == 4'd8) begin
                                    if (rx_sync2 == 1'b0) begin
                                        frame_err <= 1'b1;
                                    end
                                    tick_cnt <= tick_cnt + 1'b1;
                                end else if (tick_cnt == 4'd15) begin
                                    bit_idx  <= 4'd1; // Advance to Stop bit 2
                                    tick_cnt <= 4'd0;
                                end else begin
                                    tick_cnt <= tick_cnt + 1'b1;
                                end
                            end else begin
                                // Stop bit 2 (last stop): sample at tick 8 and return IDLE
                                if (tick_cnt == 4'd8) begin
                                    if (rx_sync2 == 1'b0) begin
                                        frame_err <= 1'b1;
                                    end
                                    rx_data  <= rx_shift_reg & data_mask;
                                    rx_valid <= 1'b1; // 1-cycle pulse
                                    // Early return to IDLE at tick 8 of last stop bit
                                    state    <= RX_IDLE;
                                    rx_busy  <= 1'b0;
                                    tick_cnt <= 4'd0;
                                    bit_idx  <= 4'd0;
                                end else begin
                                    tick_cnt <= tick_cnt + 1'b1;
                                end
                            end
                        end
                    end
                end

                default: begin
                    state   <= RX_IDLE;
                    rx_busy <= 1'b0;
                end
            endcase
        end
    end

endmodule
