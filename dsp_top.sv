module dsp_top #(
  parameter DATA_WIDTH = 16,
  parameter COEF_WIDTH = 16,
  parameter MAX_TAPS   = 8,
  parameter IIR_SECTIONS = 2,
  parameter FFT_N      = 8
)(
  input  logic clk,
  input  logic rst_n,

  // Mode selector: 00 = FIR, 01 = IIR, 10 = FFT
  input  logic [1:0] mode,

  // Input streaming
  input  logic signed [DATA_WIDTH-1:0] din_real,
  input  logic signed [DATA_WIDTH-1:0] din_imag,
  input  logic                         din_valid,
  output logic                         din_ready,

  // Output streaming
  output logic signed [DATA_WIDTH-1:0] dout_real,
  output logic signed [DATA_WIDTH-1:0] dout_imag,
  output logic                         dout_valid,

  // FIR Coefficient interface
  input  logic                         fir_coeff_wr_en,
  input  logic [$clog2(MAX_TAPS)-1:0]  fir_coeff_index,
  input  logic signed [COEF_WIDTH-1:0] fir_coeff_value,

  // IIR Coefficient interface
  input  logic                         iir_coeff_wr_en,
  input  logic [$clog2(IIR_SECTIONS)-1:0] iir_section_index,
  input  logic [2:0]                   iir_coeff_sel,
  input  logic signed [COEF_WIDTH-1:0] iir_coeff_value,

  // Control
  input  logic start,
  output logic done
);

  // FIR instance
  logic fir_valid_out;
  logic signed [DATA_WIDTH-1:0] fir_dout;

  fir #(
    .DATA_WIDTH(DATA_WIDTH), .COEF_WIDTH(COEF_WIDTH), .MAX_TAPS(MAX_TAPS)
  ) u_fir (
    .clk(clk), .rst_n(rst_n),
    .din(din_real), .din_valid(din_valid && (mode == 2'd0)),
    .dout(fir_dout), .dout_valid(fir_valid_out),
    .coeff_wr_en(fir_coeff_wr_en), .coeff_index(fir_coeff_index), .coeff_value(fir_coeff_value)
  );

  // IIR instance
  logic iir_valid_out;
  logic signed [DATA_WIDTH-1:0] iir_dout;

  iir #(
    .DATA_WIDTH(DATA_WIDTH), .COEF_WIDTH(COEF_WIDTH), .NUM_SECTIONS(IIR_SECTIONS)
  ) u_iir (
    .clk(clk), .rst_n(rst_n),
    .din(din_real), .din_valid(din_valid && (mode == 2'd1)),
    .dout(iir_dout), .dout_valid(iir_valid_out),
    .coeff_wr_en(iir_coeff_wr_en), .section_index(iir_section_index),
    .coeff_sel(iir_coeff_sel), .coeff_value(iir_coeff_value)
  );

  // FFT instance
  logic fft_valid_out;
  logic signed [DATA_WIDTH-1:0] fft_dout_real, fft_dout_imag;
  logic fft_din_ready;
  fft_radix2 #(
    .N(FFT_N), .DATA_WIDTH(DATA_WIDTH), .TWID_WIDTH(COEF_WIDTH)
  ) u_fft (
    .clk(clk), .rst_n(rst_n),
    .din_ready(fft_din_ready),
    .din_real(din_real), .din_imag(din_imag),
    .din_valid(din_valid && (mode == 2'd2)),
    .dout_real(fft_dout_real), .dout_imag(fft_dout_imag),
    .dout_valid(fft_valid_out),
    .start(start), .done(done)
  );

  always_comb begin
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

module fir #(
  parameter DATA_WIDTH = 16,
  parameter COEF_WIDTH = 16,
  parameter MAX_TAPS   = 8
)(
  input  logic                      clk,
  input  logic                      rst_n,

  // Data interface
  input  logic signed [DATA_WIDTH-1:0] din,
  input  logic                          din_valid,
  output logic                         dout_valid,
  output logic signed [DATA_WIDTH-1:0] dout,

  // Coefficient load interface
  input  logic                          coeff_wr_en,
  input  logic [$clog2(MAX_TAPS)-1:0]   coeff_index,
  input  logic signed [COEF_WIDTH-1:0]  coeff_value
);

  // Internal shift register and coefficient memory
  logic signed [DATA_WIDTH-1:0] shift_reg [0:MAX_TAPS-1];
  logic signed [COEF_WIDTH-1:0] coeffs    [0:MAX_TAPS-1];

  // Write coefficients
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < MAX_TAPS; i++) coeffs[i] <= 0;
    end else if (coeff_wr_en) begin
      coeffs[coeff_index] <= coeff_value;
    end
  end

  // FIR processing
  logic signed [DATA_WIDTH + COEF_WIDTH - 1:0] acc;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < MAX_TAPS; i++) shift_reg[i] <= 0;
      dout <= 0;
      dout_valid <= 0;
    end else if (din_valid) begin
      // Shift input
      for (int i = MAX_TAPS-1; i > 0; i--)
        shift_reg[i] <= shift_reg[i-1];
      shift_reg[0] <= din;

      // Convolution
      acc = 0;
      for (int i = 0; i < MAX_TAPS; i++)
        acc += shift_reg[i] * coeffs[i];

      dout <= acc >>> COEF_WIDTH;  
      dout_valid <= 1;
    end else begin
      dout_valid <= 0;
    end
  end

endmodule

module iir #(
  parameter DATA_WIDTH   = 16,
  parameter COEF_WIDTH   = 16,
  parameter NUM_SECTIONS = 2
)(
  input  logic clk,
  input  logic rst_n,

  // Streaming interface
  input  logic signed [DATA_WIDTH-1:0] din,
  input  logic                         din_valid,
  output logic signed [DATA_WIDTH-1:0] dout,
  output logic                         dout_valid,

  // Coefficient write interface
  input  logic                         coeff_wr_en,
  input  logic [$clog2(NUM_SECTIONS)-1:0] section_index,
  input  logic [2:0]                   coeff_sel,  // 3 bits: 000=b0, 001=b1, 010=b2, 011=a1, 100=a2
  input  logic signed [COEF_WIDTH-1:0] coeff_value
);

  // ========== Biquad state ==========
  typedef struct packed {
    logic signed [DATA_WIDTH-1:0] x1, x2;
    logic signed [DATA_WIDTH-1:0] y1, y2;
  } state_t;

  state_t states[NUM_SECTIONS];

  // ========== Coefficient Storage ==========
  typedef struct packed {
    logic signed [COEF_WIDTH-1:0] b0, b1, b2;
    logic signed [COEF_WIDTH-1:0] a1, a2;
  } coef_t;

  coef_t coeffs[NUM_SECTIONS];

  // Write coefficients
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int s = 0; s < NUM_SECTIONS; s++) begin
        coeffs[s].b0 <= 16'sd4096;
        coeffs[s].b1 <= 16'sd8192;
        coeffs[s].b2 <= 16'sd4096;
        coeffs[s].a1 <= 16'sd8192;
        coeffs[s].a2 <= 16'sd4096;
      end
    end else if (coeff_wr_en) begin
      case (coeff_sel)
        3'd0: coeffs[section_index].b0 <= coeff_value;
        3'd1: coeffs[section_index].b1 <= coeff_value;
        3'd2: coeffs[section_index].b2 <= coeff_value;
        3'd3: coeffs[section_index].a1 <= coeff_value;
        3'd4: coeffs[section_index].a2 <= coeff_value;
        default: ; 
      endcase
    end
  end

  // ========== Processing Logic ==========
  logic signed [DATA_WIDTH-1:0] section_in, section_out;
  logic signed [DATA_WIDTH+COEF_WIDTH-1:0] acc;

  assign dout = section_out;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int s = 0; s < NUM_SECTIONS; s++) begin
        states[s].x1 <= 0; states[s].x2 <= 0;
        states[s].y1 <= 0; states[s].y2 <= 0;
      end
      dout_valid <= 0;
    end else if (din_valid) begin
      section_in = din;

      for (int s = 0; s < NUM_SECTIONS; s++) begin
        acc =  coeffs[s].b0 * section_in +
               coeffs[s].b1 * states[s].x1 +
               coeffs[s].b2 * states[s].x2 -
               coeffs[s].a1 * states[s].y1 -
               coeffs[s].a2 * states[s].y2;

        section_out = acc >>> COEF_WIDTH;

        // Update state
        states[s].x2 <= states[s].x1;
        states[s].x1 <= section_in;
        states[s].y2 <= states[s].y1;
        states[s].y1 <= section_out;

        section_in = section_out; 
      end

      dout_valid <= 1;
    end else begin
      dout_valid <= 0;
    end
  end

endmodule

module fft_radix2 #(
  parameter N = 8,
  parameter DATA_WIDTH = 16,
  parameter TWID_WIDTH = 16
)(
  input  logic clk,
  input  logic rst_n,

  // Input streaming interface
  input  logic signed [DATA_WIDTH-1:0] din_real,
  input  logic signed [DATA_WIDTH-1:0] din_imag,
  input  logic                         din_valid,
  output logic                         din_ready,

  // Output streaming interface
  output logic signed [DATA_WIDTH-1:0] dout_real,
  output logic signed [DATA_WIDTH-1:0] dout_imag,
  output logic                         dout_valid,

  // Control
  input  logic start,
  output logic done
);

  localparam STAGES = $clog2(N);

  // Input buffers
  logic signed [DATA_WIDTH-1:0] real_buf [0:N-1];
  logic signed [DATA_WIDTH-1:0] imag_buf [0:N-1];

  logic [$clog2(N)-1:0] sample_idx;
  logic [$clog2(N)-1:0] output_idx;

  logic loading, computing, outputting;

  assign din_ready = loading;
  assign dout_real = real_buf[output_idx];
  assign dout_imag = imag_buf[output_idx];

  // Twiddle ROM 
  function automatic [2*TWID_WIDTH-1:0] get_twiddle(input int k, input int N);
    real angle = -2.0 * 3.14159265358979 * k / N;
    int cos_val = $rtoi($cos(angle) * (1 << (TWID_WIDTH-1)));
    int sin_val = $rtoi($sin(angle) * (1 << (TWID_WIDTH-1)));
    return {cos_val[TWID_WIDTH-1:0], sin_val[TWID_WIDTH-1:0]};
  endfunction

  // Control FSM
  typedef enum logic [1:0] {IDLE, LOAD, COMPUTE, OUTPUT} state_t;
  state_t state;

  // FFT computation
  task automatic fft_compute();
    logic signed [DATA_WIDTH+TWID_WIDTH-1:0] tempr, tempi;
    logic signed [TWID_WIDTH-1:0] wr, wi;

    for (int s = 0; s < STAGES; s++) begin
      int m = 1 << (s + 1);
      int half_m = m >> 1;
      for (int k = 0; k < N; k += m) begin
        for (int j = 0; j < half_m; j++) begin
          int index1 = k + j;
          int index2 = k + j + half_m;
          {wr, wi} = get_twiddle(j << (STAGES - s - 1), N);
          tempr = (real_buf[index2] * wr - imag_buf[index2] * wi) >>> (TWID_WIDTH - 1);
          tempi = (real_buf[index2] * wi + imag_buf[index2] * wr) >>> (TWID_WIDTH - 1);

          // Butterfly
          real_buf[index2] <= real_buf[index1] - tempr;
          imag_buf[index2] <= imag_buf[index1] - tempi;
          real_buf[index1] <= real_buf[index1] + tempr;
          imag_buf[index1] <= imag_buf[index1] + tempi;
        end
      end
    end
  endtask

  // State machine
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= IDLE;
      sample_idx <= 0;
      output_idx <= 0;
      loading <= 0;
      computing <= 0;
      outputting <= 0;
      done <= 0;
      dout_valid <= 0;
    end else begin
      case (state)
        IDLE: begin
          if (start) begin
            loading <= 1;
            sample_idx <= 0;
            state <= LOAD;
            done <= 0;
          end
        end

        LOAD: begin
          if (din_valid && loading) begin
            real_buf[sample_idx] <= din_real;
            imag_buf[sample_idx] <= din_imag;
            sample_idx <= sample_idx + 1;
            if (sample_idx == N - 1) begin
              loading <= 0;
              state <= COMPUTE;
            end
          end
        end

        COMPUTE: begin
          fft_compute();
          state <= OUTPUT;
          output_idx <= 0;
        end

        OUTPUT: begin
          dout_valid <= 1;
          output_idx <= output_idx + 1;
          if (output_idx == N - 1) begin
            state <= IDLE;
            dout_valid <= 0;
            done <= 1;
          end
        end
      endcase
    end
  end

endmodule
