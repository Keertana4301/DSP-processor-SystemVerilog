module fir #(
  parameter DATA_WIDTH = 16,
  parameter COEF_WIDTH = 16,
  parameter MAX_TAPS   = 8
)(
  input  wire clk,
  input  wire rst_n,

  input  wire signed [DATA_WIDTH-1:0] din,
  input  wire din_valid,
  output reg  dout_valid,
  output reg signed [DATA_WIDTH-1:0] dout,

  input  wire coeff_wr_en,
  input  wire [$clog2(MAX_TAPS)-1:0] coeff_index,
  input  wire signed [COEF_WIDTH-1:0] coeff_value
);

  reg signed [DATA_WIDTH-1:0] shift_reg [0:MAX_TAPS-1];
  reg signed [COEF_WIDTH-1:0] coeffs    [0:MAX_TAPS-1];

  reg signed [DATA_WIDTH + COEF_WIDTH - 1:0] acc;

  integer i;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (i = 0; i < MAX_TAPS; i = i + 1) begin
        shift_reg[i] <= 0;
        coeffs[i] <= 0;
      end
      dout <= 0;
      dout_valid <= 0;
    end else begin
      if (coeff_wr_en) begin
        coeffs[coeff_index] <= coeff_value;
      end

      if (din_valid) begin
        for (i = MAX_TAPS-1; i > 0; i = i - 1)
          shift_reg[i] <= shift_reg[i-1];
        shift_reg[0] <= din;

        acc = 0;
        for (i = 0; i < MAX_TAPS; i = i + 1)
          acc = acc + shift_reg[i] * coeffs[i];

        dout <= acc >>> COEF_WIDTH;
        dout_valid <= 1;
      end else begin
        dout_valid <= 0;
      end
    end
  end

endmodule
