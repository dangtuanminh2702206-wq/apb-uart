// =============================================================================
// Module: baud_gen
// Description: Fractional baud rate generator with 16x oversampling tick.
//              Uses a 4-bit fractional accumulator: D = div_int + div_frac/16.
//              Generates a 1-pclk-cycle tick enable pulse every D cycles on average.
// Fixes Applied:
//   - Zero-extended arithmetic to prevent lint width mismatch (B5)
//   - Safe rollover using >= to prevent 4096-cycle rollover hang when divisor decreases (B1)
// =============================================================================

module baud_gen (
    input  wire        pclk,
    input  wire        presetn,
    input  wire [11:0] div_int,
    input  wire [3:0]  div_frac,
    output reg         tick
);

    reg [11:0] cnt; // Sub-baud counter
    reg [3:0]  acc; // Fractional accumulator

    // Explicit zero-extended widths for linting cleanliness (B5)
    wire [4:0]  sum   = {1'b0, acc} + {1'b0, div_frac};
    wire        carry = sum[4];
    wire [12:0] L     = {1'b0, div_int} + {12'd0, carry};

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            cnt  <= 12'd0;
            acc  <= 4'd0;
            tick <= 1'b0;
        end else begin
            // Safe comparison (>=) ensures no multi-thousand cycle rollover delay
            // when divisor is reduced while counter is already above the new limit (B1)
            if (div_int == 12'd0) begin
                // Defensive standalone behavior. APB rejects a zero divisor.
                tick <= 1'b0;
                cnt  <= 12'd0;
                acc  <= 4'd0;
            end else if ({1'b0, cnt} >= (L - 13'd1)) begin
                tick <= 1'b1;
                cnt  <= 12'd0;
                acc  <= sum[3:0]; // (acc + div_frac) mod 16
            end else begin
                tick <= 1'b0;
                cnt  <= cnt + 12'd1;
            end
        end
    end

endmodule
