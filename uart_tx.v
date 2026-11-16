// =============================================================================
// Module: uart_tx
// Description: UART Transmitter with PISO shift register, configurable frame
//              (5-8 data bits, parity Even/Odd, 1-2 stop bits), hardware CTS
//              handshake, and level-status tx_done flag.
// Reference: uart_tx_rx_spec_v2.md, HD-APB-UART-tuong-tac.html
// Standard: Verilog-2001, synchronous to pclk, active-low presetn
// =============================================================================

module uart_tx (
    input  wire       pclk,        // System clock (25 MHz)
    input  wire       presetn,     // Asynchronous active-low reset
    input  wire       tick,        // Baud tick enable from baud_gen (1 pclk wide)
    input  wire [7:0] tx_data,     // Data byte from TXDATA register
    input  wire       start,       // 1-cycle transmit request pulse
    input  wire       cts_s,       // Synchronized CTS input (active-low: 0 = clear to send)
    input  wire       tx_dis,      // 1: Disable TX (block new transmissions)
    input  wire [1:0] data_bits,   // 00: 5-bit, 01: 6-bit, 10: 7-bit, 11: 8-bit
    input  wire       parity_en,   // 1: Enable parity
    input  wire       parity_type, // 0: Odd, 1: Even (spec v2.0 & HTML line 442)
    input  wire       stop_bits,   // 0: 1 stop bit, 1: 2 stop bits
    output reg        tx,          // Serial output line (idle = 1)
    output reg        tx_done      // Level status: 1 when idle/done, 0 when transmitting
);

    // -------------------------------------------------------------------------
    // FSM State Encoding
    // Fix U1: tx_done drops to 0 as soon as start is accepted (including WAIT_CTS)
    // Fix U2: TX_START_WAIT ensures Start bit starts strictly on a tick boundary
    // -------------------------------------------------------------------------
    localparam [2:0]
        TX_IDLE       = 3'd0,
        TX_WAIT_CTS   = 3'd1,
        TX_START_WAIT = 3'd2,
        TX_START      = 3'd3,
        TX_DATA       = 3'd4,
        TX_PARITY     = 3'd5,
        TX_STOP       = 3'd6;

    reg [2:0] state;

    // -------------------------------------------------------------------------
    // Internal Datapath Registers
    // -------------------------------------------------------------------------
    reg [3:0] tick_cnt;         // Sub-bit timing counter (0..15)
    reg [3:0] bit_idx;          // Frame bit index / data bit counter
    reg [7:0] shift_reg;        // PISO shift register
    reg       first_data_bit;   // Only bit 0 is needed outside the shift register
    reg       frame_parity_q;   // Calculated parity bit to transmit
    reg [3:0] frame_len_q;      // Data length (5, 6, 7, 8)
    reg       frame_par_en_q;   // Frame parity enable
    reg       frame_stop2_q;    // Frame 2-stop-bit flag

    // -------------------------------------------------------------------------
    // Combinational Parity & Data Mask Calculation
    // -------------------------------------------------------------------------
    wire [7:0] data_mask = (data_bits == 2'b00) ? 8'h1F :
                           (data_bits == 2'b01) ? 8'h3F :
                           (data_bits == 2'b10) ? 8'h7F : 8'hFF;

    wire [7:0] masked_tx_data = tx_data & data_mask;
    wire       p_even         = ^masked_tx_data;
    wire       p_odd          = ~p_even;
    wire       calc_parity    = parity_type ? p_even : p_odd;

    wire [3:0] current_len    = 4'd5 + {2'b00, data_bits};

    // -------------------------------------------------------------------------
    // FSM & Datapath Sequential Logic
    // -------------------------------------------------------------------------
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            state          <= TX_IDLE;
            tx             <= 1'b1;
            tx_done        <= 1'b1; // Reset status = 1 (idle/ready)
            tick_cnt       <= 4'd0;
            bit_idx        <= 4'd0;
            shift_reg      <= 8'h00;
            first_data_bit <= 1'b0;
            frame_parity_q <= 1'b0;
            frame_len_q    <= 4'd8;
            frame_par_en_q <= 1'b0;
            frame_stop2_q  <= 1'b0;
        end else begin
            case (state)
                // -------------------------------------------------------------
                // TX_IDLE: Awaiting transmission request
                // -------------------------------------------------------------
                TX_IDLE: begin
                    tx      <= 1'b1;
                    tx_done <= 1'b1;
                    tick_cnt<= 4'd0;
                    bit_idx <= 4'd0;

                    if (start && !tx_dis) begin
                        // Capture frame configuration and data
                        first_data_bit <= masked_tx_data[0];
                        frame_len_q    <= current_len;
                        frame_par_en_q <= parity_en;
                        frame_parity_q <= calc_parity;
                        frame_stop2_q  <= stop_bits;
                        shift_reg      <= {1'b0, masked_tx_data[7:1]};

                        // U1 Fix: tx_done drops to 0 immediately upon start acceptance
                        tx_done <= 1'b0;
                        tx      <= 1'b1;

                        if (cts_s) begin
                            // CTS busy: wait in TX_WAIT_CTS
                            state <= TX_WAIT_CTS;
                        end else if (tick) begin
                            // Exactly at tick boundary: launch Start bit immediately
                            state    <= TX_START;
                            tx       <= 1'b0;
                            tick_cnt <= 4'd0;
                        end else begin
                            // U2 Fix: Wait for next tick boundary before asserting tx=0
                            state    <= TX_START_WAIT;
                        end
                    end
                end

                // -------------------------------------------------------------
                // TX_WAIT_CTS: Waiting for CTS handshake to clear (cts_s == 0)
                // -------------------------------------------------------------
                TX_WAIT_CTS: begin
                    tx      <= 1'b1;
                    tx_done <= 1'b0; // U1 Fix: Keep tx_done low while waiting for CTS
                    if (!cts_s) begin
                        if (tick) begin
                            state    <= TX_START;
                            tx       <= 1'b0;
                            tick_cnt <= 4'd0;
                            bit_idx  <= 4'd0;
                        end else begin
                            state    <= TX_START_WAIT;
                        end
                    end
                end

                // -------------------------------------------------------------
                // TX_START_WAIT: Align Start bit to the next tick boundary
                // -------------------------------------------------------------
                TX_START_WAIT: begin
                    tx      <= 1'b1;
                    tx_done <= 1'b0;
                    if (cts_s) begin
                        state <= TX_WAIT_CTS;
                    end else if (tick) begin
                        state    <= TX_START;
                        tx       <= 1'b0; // Pull tx low on tick edge
                        tick_cnt <= 4'd0;
                        bit_idx  <= 4'd0;
                    end
                end

                // -------------------------------------------------------------
                // TX_START: Transmitting Start bit (logic 0 for 16 ticks)
                // -------------------------------------------------------------
                TX_START: begin
                    tx      <= 1'b0;
                    tx_done <= 1'b0;
                    if (tick) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;
                            bit_idx  <= 4'd0;
                            state    <= TX_DATA;
                            tx       <= first_data_bit; // First data bit (LSB)
                        end else begin
                            tick_cnt <= tick_cnt + 1'b1;
                        end
                    end
                end

                // -------------------------------------------------------------
                // TX_DATA: Transmitting N data bits (LSB first, 16 ticks/bit)
                // -------------------------------------------------------------
                TX_DATA: begin
                    tx_done <= 1'b0;
                    if (tick) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;
                            if (bit_idx < frame_len_q - 1'b1) begin
                                bit_idx   <= bit_idx + 1'b1;
                                tx        <= shift_reg[0];
                                shift_reg <= {1'b0, shift_reg[7:1]};
                            end else begin
                                // Finished all N data bits
                                bit_idx <= 4'd0;
                                if (frame_par_en_q) begin
                                    state <= TX_PARITY;
                                    tx    <= frame_parity_q;
                                end else begin
                                    state <= TX_STOP;
                                    tx    <= 1'b1; // Stop bit is logic 1
                                end
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 1'b1;
                        end
                    end
                end

                // -------------------------------------------------------------
                // TX_PARITY: Transmitting Parity bit (16 ticks)
                // -------------------------------------------------------------
                TX_PARITY: begin
                    tx      <= frame_parity_q;
                    tx_done <= 1'b0;
                    if (tick) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;
                            bit_idx  <= 4'd0;
                            state    <= TX_STOP;
                            tx       <= 1'b1;
                        end else begin
                            tick_cnt <= tick_cnt + 1'b1;
                        end
                    end
                end

                // -------------------------------------------------------------
                // TX_STOP: Transmitting Stop bit(s) (1 or 2 stop bits, logic 1)
                // -------------------------------------------------------------
                TX_STOP: begin
                    tx      <= 1'b1;
                    tx_done <= 1'b0;
                    if (tick) begin
                        if (tick_cnt == 4'd15) begin
                            if (frame_stop2_q && (bit_idx == 4'd0)) begin
                                // Finished Stop bit 1; proceed to Stop bit 2
                                bit_idx  <= 4'd1;
                                tick_cnt <= 4'd0;
                            end else begin
                                // Frame complete! Return to IDLE
                                state    <= TX_IDLE;
                                tx_done  <= 1'b1;
                                tick_cnt <= 4'd0;
                                bit_idx  <= 4'd0;
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 1'b1;
                        end
                    end
                end

                default: begin
                    state   <= TX_IDLE;
                    tx      <= 1'b1;
                    tx_done <= 1'b1;
                end
            endcase
        end
    end

endmodule
