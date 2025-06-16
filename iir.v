module iir #(
  parameter DATA_WIDTH = 16,
  parameter COEF_WIDTH = 16,
  parameter NUM_SECTIONS = 2
)(
  input  wire clk,
  input  wire rst_n,

  input  wire signed [DATA_WIDTH-1:0] din,
  input  wire din_valid,
  output reg  signed [DATA_WIDTH-1:0] dout,
  output reg  dout_valid,

  input  wire coeff_wr_en,
  input  wire [$clog2(NUM_SECTIONS)-1:0] section_index,
  input  wire [2:0] coeff_sel,
  input  wire signed [COEF_WIDTH-1:0] coeff_value
);

  reg signed [DATA_WIDTH-1:0] x1 [0:NUM_SECTIONS-1];
  reg signed [DATA_WIDTH-1:0] x2 [0:NUM_SECTIONS-1];
  reg signed [DATA_WIDTH-1:0] y1 [0:NUM_SECTIONS-1];
  reg signed [DATA_WIDTH-1:0] y2 [0:NUM_SECTIONS-1];

  reg signed [COEF_WIDTH-1:0] b0 [0:NUM_SECTIONS-1];
  reg signed [COEF_WIDTH-1:0] b1 [0:NUM_SECTIONS-1];
  reg signed [COEF_WIDTH-1:0] b2 [0:NUM_SECTIONS-1];
  reg signed [COEF_WIDTH-1:0] a1 [0:NUM_SECTIONS-1];
  reg signed [COEF_WIDTH-1:0] a2 [0:NUM_SECTIONS-1];

  integer s;
  reg signed [DATA_WIDTH + COEF_WIDTH - 1:0] acc;
  reg signed [DATA_WIDTH-1:0] section_in, section_out;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (s = 0; s < NUM_SECTIONS; s = s + 1) begin
        x1[s] <= 0; x2[s] <= 0;
        y1[s] <= 0; y2[s] <= 0;
        b0[s] <= 16'sd4096;
        b1[s] <= 16'sd8192;
        b2[s] <= 16'sd4096;
        a1[s] <= 16'sd8192;
        a2[s] <= 16'sd4096;
      end
      dout_valid <= 0;
    end else begin
      if (coeff_wr_en) begin
        case (coeff_sel)
          3'd0: b0[section_index] <= coeff_value;
          3'd1: b1[section_index] <= coeff_value;
          3'd2: b2[section_index] <= coeff_value;
          3'd3: a1[section_index] <= coeff_value;
          3'd4: a2[section_index] <= coeff_value;
        endcase
      end

      if (din_valid) begin
        section_in = din;
        for (s = 0; s < NUM_SECTIONS; s = s + 1) begin
          acc =  b0[s] * section_in +
                 b1[s] * x1[s] +
                 b2[s] * x2[s] -
                 a1[s] * y1[s] -
                 a2[s] * y2[s];

          section_out = acc >>> COEF_WIDTH;

          x2[s] <= x1[s];
          x1[s] <= section_in;
          y2[s] <= y1[s];
          y1[s] <= section_out;

          section_in = section_out;
        end

        dout <= section_out;
        dout_valid <= 1;
      end else begin
        dout_valid <= 0;
      end
    end
  end

endmodule
