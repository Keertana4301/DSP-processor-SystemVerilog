module dsp_top #(
  parameter DATA_WIDTH = 16,
  parameter COEF_WIDTH = 16,
  parameter MAX_TAPS   = 8,
  parameter IIR_SECTIONS = 2,
  parameter FFT_N      = 8
)(
  input  wire clk,
  input  wire rst_n,

  input  wire [1:0] mode,

  input  wire signed [DATA_WIDTH-1:0] din_real,
  input  wire signed [DATA_WIDTH-1:0] din_imag,
  input  wire                         din_valid,
  output reg                          din_ready,

  output reg signed [DATA_WIDTH-1:0] dout_real,
  output reg signed [DATA_WIDTH-1:0] dout_imag,
  output reg                          dout_valid,

  input  wire                         fir_coeff_wr_en,
  input  wire [$clog2(MAX_TAPS)-1:0]  fir_coeff_index,
  input  wire signed [COEF_WIDTH-1:0] fir_coeff_value,

  input  wire                         iir_coeff_wr_en,
  input  wire [$clog2(IIR_SECTIONS)-1:0] iir_section_index,
  input  wire [2:0]                   iir_coeff_sel,
  input  wire signed [COEF_WIDTH-1:0] iir_coeff_value,

  input  wire start,
  output wire done
);

  wire signed [DATA_WIDTH-1:0] fir_dout;
  wire fir_valid_out;

  fir #(
    .DATA_WIDTH(DATA_WIDTH),
    .COEF_WIDTH(COEF_WIDTH),
    .MAX_TAPS(MAX_TAPS)
  ) u_fir (
    .clk(clk),
    .rst_n(rst_n),
    .din(din_real),
    .din_valid(din_valid && (mode == 2'd0)),
    .dout_valid(fir_valid_out),
    .dout(fir_dout),
    .coeff_wr_en(fir_coeff_wr_en),
    .coeff_index(fir_coeff_index),
    .coeff_value(fir_coeff_value)
  );

  wire signed [DATA_WIDTH-1:0] iir_dout;
  wire iir_valid_out;

  iir #(
    .DATA_WIDTH(DATA_WIDTH),
    .COEF_WIDTH(COEF_WIDTH),
    .NUM_SECTIONS(IIR_SECTIONS)
  ) u_iir (
    .clk(clk),
    .rst_n(rst_n),
    .din(din_real),
    .din_valid(din_valid && (mode == 2'd1)),
    .dout(iir_dout),
    .dout_valid(iir_valid_out),
    .coeff_wr_en(iir_coeff_wr_en),
    .section_index(iir_section_index),
    .coeff_sel(iir_coeff_sel),
    .coeff_value(iir_coeff_value)
  );

  wire signed [DATA_WIDTH-1:0] fft_dout_real, fft_dout_imag;
  wire fft_valid_out;
  wire fft_din_ready;

  fft_radix2 #(
    .N(FFT_N),
    .DATA_WIDTH(DATA_WIDTH),
    .TWID_WIDTH(COEF_WIDTH)
  ) u_fft (
    .clk(clk),
    .rst_n(rst_n),
    .din_real(din_real),
    .din_imag(din_imag),
    .din_valid(din_valid && (mode == 2'd2)),
    .din_ready(fft_din_ready),
    .dout_real(fft_dout_real),
    .dout_imag(fft_dout_imag),
    .dout_valid(fft_valid_out),
    .start(start),
    .done(done)
  );

  always @(*) begin
    case (mode)
      2'd0: begin
        dout_real  = fir_dout;
        dout_imag  = 0;
        dout_valid = fir_valid_out;
        din_ready  = 1;
      end
      2'd1: begin
        dout_real  = iir_dout;
        dout_imag  = 0;
        dout_valid = iir_valid_out;
        din_ready  = 1;
      end
      2'd2: begin
        dout_real  = fft_dout_real;
        dout_imag  = fft_dout_imag;
        dout_valid = fft_valid_out;
        din_ready  = fft_din_ready;
      end
      default: begin
        dout_real  = 0;
        dout_imag  = 0;
        dout_valid = 0;
        din_ready  = 0;
      end
    endcase
  end

endmodule
