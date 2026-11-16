// SPI mode-0, MSB-first, fixed 56-bit frame to an APB4 master.
// Source contract: HD-APB-UART-tuong-tac.html, section 9.
// All state is clocked by pclk; spi_sclk is synchronized data, never a clock.
`default_nettype none

module spi2apb (
    input  wire        pclk,
    input  wire        presetn,
    input  wire        spi_cs_n,
    input  wire        spi_sclk,
    input  wire        spi_mosi,
    output wire        spi_miso,
    output wire        psel,
    output wire        penable,
    output reg         pwrite,
    output reg  [11:0] paddr,
    output reg  [3:0]  pstrb,
    output reg  [31:0] pwdata,
    input  wire        pready,
    input  wire        pslverr,
    input  wire [31:0] prdata
);
    (* async_reg = "true" *) reg cs_meta_q, cs_sync_q;
    (* async_reg = "true" *) reg sclk_meta_q, sclk_sync_q;
    (* async_reg = "true" *) reg mosi_meta_q, mosi_sync_q;
    reg cs_prev_q, sclk_prev_q;

    wire cs_fall   = cs_prev_q && !cs_sync_q;
    wire sclk_rise = !sclk_prev_q && sclk_sync_q;
    wire sclk_fall = sclk_prev_q && !sclk_sync_q;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            cs_meta_q   <= 1'b1;
            cs_sync_q   <= 1'b1;
            cs_prev_q   <= 1'b1;
            sclk_meta_q <= 1'b0;
            sclk_sync_q <= 1'b0;
            sclk_prev_q <= 1'b0;
            mosi_meta_q <= 1'b0;
            mosi_sync_q <= 1'b0;
        end else begin
            cs_meta_q   <= spi_cs_n;
            cs_sync_q   <= cs_meta_q;
            cs_prev_q   <= cs_sync_q;
            sclk_meta_q <= spi_sclk;
            sclk_sync_q <= sclk_meta_q;
            sclk_prev_q <= sclk_sync_q;
            mosi_meta_q <= spi_mosi;
            mosi_sync_q <= mosi_meta_q;
        end
    end

    localparam [2:0] SPI_IDLE    = 3'd0,
                     SPI_CMD     = 3'd1,
                     SPI_TURN    = 3'd2,
                     SPI_DATA    = 3'd3,
                     SPI_STATUS  = 3'd4,
                     SPI_WAIT_CS = 3'd5;
    reg [2:0]  spi_state_q;
    // Index of the next MOSI sample (0..55); saturates after the last bit.
    reg [5:0]  bit_count_q;
    reg [7:0]  command_shift_q, command_q;
    reg [31:0] write_shift_q;
    reg        cmd_error_q;
    reg        read_req_q, write_req_q;
    wire [7:0] command_next = {command_shift_q[6:0], mosi_sync_q};

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            spi_state_q     <= SPI_IDLE;
            bit_count_q     <= 6'd0;
            command_shift_q <= 8'd0;
            command_q       <= 8'd0;
            write_shift_q   <= 32'd0;
            cmd_error_q     <= 1'b0;
            read_req_q      <= 1'b0;
            write_req_q     <= 1'b0;
        end else begin
            read_req_q  <= 1'b0;
            write_req_q <= 1'b0;
            if (cs_sync_q) begin
                spi_state_q     <= SPI_IDLE;
                bit_count_q     <= 6'd0;
                command_shift_q <= 8'd0;
                command_q       <= 8'd0;
                write_shift_q   <= 32'd0;
                cmd_error_q     <= 1'b0;
            end else begin
                if (spi_state_q == SPI_IDLE) spi_state_q <= SPI_CMD;
                // Consuming bit 0 here also handles selection and the first
                // synchronized SCLK rise arriving on the same pclk edge.
                if (sclk_rise && spi_state_q != SPI_WAIT_CS) begin
                    if (bit_count_q < 6'd55)
                        bit_count_q <= bit_count_q + 6'd1;

                    if (bit_count_q < 6'd8)
                        command_shift_q <= command_next;

                    if (bit_count_q == 6'd7) begin
                        command_q   <= command_next;
                        cmd_error_q <= |command_next[6:4];
                        spi_state_q <= SPI_TURN;
                        if (!(|command_next[6:4]) && !command_next[7])
                            read_req_q <= 1'b1;
                    end
                    if (bit_count_q == 6'd15) spi_state_q <= SPI_DATA;
                    if (bit_count_q >= 6'd16 && bit_count_q <= 6'd47)
                        write_shift_q <= {write_shift_q[30:0], mosi_sync_q};

                    if (bit_count_q == 6'd47) begin
                        spi_state_q <= SPI_STATUS;
                        if (!cmd_error_q && command_q[7]) write_req_q <= 1'b1;
                    end
                    if (bit_count_q == 6'd55) spi_state_q <= SPI_WAIT_CS;
                end
            end
        end
    end

    localparam [1:0] APB_IDLE   = 2'd0,
                     APB_SETUP  = 2'd1,
                     APB_ACCESS = 2'd2;
    reg [1:0]  apb_state_q;
    reg [31:0] read_data_q;
    reg        apb_error_q;
    assign psel    = (apb_state_q != APB_IDLE);
    assign penable = (apb_state_q == APB_ACCESS);

    // The APB request pulse is consumed on the following pclk edge, after
    // command_q / write_shift_q include the final received bit. CS does not
    // cancel a request already issued, including a read's register side effect.
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            apb_state_q <= APB_IDLE;
            paddr       <= 12'd0;
            pwrite      <= 1'b0;
            pstrb       <= 4'd0;
            pwdata      <= 32'd0;
            read_data_q <= 32'd0;
            apb_error_q <= 1'b0;
        end else begin
            case (apb_state_q)
                APB_IDLE: begin
                    if (read_req_q || write_req_q) begin
                        paddr       <= {6'd0, command_q[3:0], 2'b00};
                        pwrite      <= write_req_q;
                        pstrb       <= write_req_q ? 4'hF : 4'h0;
                        pwdata      <= write_req_q ? write_shift_q : 32'd0;
                        apb_state_q <= APB_SETUP;
                    end
                end
                APB_SETUP: apb_state_q <= APB_ACCESS;
                APB_ACCESS: begin
                    if (pready) begin
                        if (!pwrite) read_data_q <= prdata;
                        apb_error_q <= pslverr;
                        apb_state_q <= APB_IDLE;
                    end
                end
                default: apb_state_q <= APB_IDLE;
            endcase
            // Frame-local response state; core pready=1 guarantees completion
            // well before the next complete command can issue another request.
            if (cs_fall) begin
                read_data_q <= 32'd0;
                apb_error_q <= 1'b0;
            end
        end
    end

    reg miso_q;
    wire [5:0] read_bit_offset = 6'd47 - bit_count_q;
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            miso_q <= 1'b0;
        end else if (cs_sync_q || spi_state_q == SPI_WAIT_CS) begin
            miso_q <= 1'b0;
        end else if (sclk_fall) begin
            // bit_count_q already indexes the NEXT rising-edge sample.
            if (bit_count_q >= 6'd16 && bit_count_q <= 6'd47 &&
                !command_q[7] && !cmd_error_q)
                miso_q <= read_data_q[read_bit_offset[4:0]];
            else if (bit_count_q == 6'd54)
                miso_q <= cmd_error_q;
            else if (bit_count_q == 6'd55)
                miso_q <= apb_error_q;
            else
                miso_q <= 1'b0;
        end
    end
    // Immediate deselect/reset behavior, independent of synchronizer latency.
    assign spi_miso = (!spi_cs_n && presetn) ? miso_q : 1'b0;
    // Bits shifted out or reserved by the command protocol.
    wire unused_spi_bits = &{1'b0, command_shift_q[7], command_q[6:4], read_bit_offset[5]};
endmodule

`default_nettype wire
