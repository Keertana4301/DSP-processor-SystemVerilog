`timescale 1ns / 1ps

module dsp_tb;

  // Parameters
  parameter DATA_WIDTH = 16;
  parameter COEF_WIDTH = 16;
  parameter MAX_TAPS = 8;
  parameter IIR_SECTIONS = 2;
  parameter FFT_N = 8;

  // Clock and reset
  logic clk = 0;
  logic rst_n = 0;
  always #5 clk = ~clk;

  // Inputs
  logic [1:0] mode;
  logic start;
  logic signed [DATA_WIDTH-1:0] din_real, din_imag;
  logic din_valid;

  // Outputs
  logic signed [DATA_WIDTH-1:0] dout_real, dout_imag;
  logic dout_valid, din_ready, done;

  // Coeff interfaces
  logic fir_coeff_wr_en;
  logic [$clog2(MAX_TAPS)-1:0] fir_coeff_index;
  logic signed [COEF_WIDTH-1:0] fir_coeff_value;

  logic iir_coeff_wr_en;
  logic [$clog2(IIR_SECTIONS)-1:0] iir_section_index;
  logic [2:0] iir_coeff_sel;
  logic signed [COEF_WIDTH-1:0] iir_coeff_value;

  // Instantiate DUT
  dsp_top #(
    .DATA_WIDTH(DATA_WIDTH),
    .COEF_WIDTH(COEF_WIDTH),
    .MAX_TAPS(MAX_TAPS),
    .IIR_SECTIONS(IIR_SECTIONS),
    .FFT_N(FFT_N)
  ) dut (
    .clk(clk), .rst_n(rst_n),
    .mode(mode), .start(start),
    .din_real(din_real), .din_imag(din_imag),
    .din_valid(din_valid), .din_ready(din_ready),
    .dout_real(dout_real), .dout_imag(dout_imag),
    .dout_valid(dout_valid), .done(done),
    .fir_coeff_wr_en(fir_coeff_wr_en),
    .fir_coeff_index(fir_coeff_index),
    .fir_coeff_value(fir_coeff_value),
    .iir_coeff_wr_en(iir_coeff_wr_en),
    .iir_section_index(iir_section_index),
    .iir_coeff_sel(iir_coeff_sel),
    .iir_coeff_value(iir_coeff_value)
  );
  initial begin
    $display("==== DSP Testbench Starting ====");
    rst_n = 0; start = 0; din_valid = 0;
    fir_coeff_wr_en = 0; iir_coeff_wr_en = 0;
    #20;
    rst_n = 1;

    // ==== FIR TEST ====
    $display("[FIR] Loading coefficients...");
    mode = 2'd0;
    for (int i = 0; i < MAX_TAPS; i++) begin
      fir_coeff_index = i;
      fir_coeff_value = 16'sd256;
      fir_coeff_wr_en = 1;
      #10;
    end
    fir_coeff_wr_en = 0;

    $display("[FIR] Sending input samples...");
    for (int i = 0; i < 16; i++) begin
      din_real = i;
      din_imag = 0;
      din_valid = 1;
      start = (i == 0);
      #10;
    end
    din_valid = 0;
    #50;

    // ==== IIR TEST ====
    $display("[IIR] Loading coefficients...");
    mode = 2'd1;
    for (int s = 0; s < IIR_SECTIONS; s++) begin
      for (int c = 0; c < 5; c++) begin
        iir_section_index = s;
        iir_coeff_sel = c;
        iir_coeff_value = (c < 3) ? 16'sd4096 : 16'sd8192;
        iir_coeff_wr_en = 1;
        #10;
      end
    end
    iir_coeff_wr_en = 0;

    $display("[IIR] Sending input samples...");
    for (int i = 0; i < 16; i++) begin
      din_real = i * 100;
      din_valid = 1;
      start = (i == 0);
      #10;
    end
    din_valid = 0;
    #50;

    // ==== FFT TEST ====
    $display("[FFT] Sending input samples...");
    mode = 2'd2;
    for (int i = 0; i < FFT_N; i++) begin
      din_real = i * 100;
      din_imag = 0;
      din_valid = 1;
      start = (i == 0);
      #10;
    end
    din_valid = 0;
    #100;

    $display("==== DSP Testbench Done ====");
    $finish;
  end
  initial begin
    $dumpfile("dsp_waveform.vcd");
    $dumpvars(0, dsp_tb);
  end

endmodule

