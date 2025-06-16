module fft_radix2 #(
  parameter N = 8,
  parameter DATA_WIDTH = 16,
  parameter TWID_WIDTH = 16
)(
  input  wire clk,
  input  wire rst_n,

  input  wire signed [DATA_WIDTH-1:0] din_real,
  input  wire signed [DATA_WIDTH-1:0] din_imag,
  input  wire din_valid,
  output reg  din_ready,

  output reg signed [DATA_WIDTH-1:0] dout_real,
  output reg signed [DATA_WIDTH-1:0] dout_imag,
  output reg dout_valid,

  input  wire start,
  output reg done
);

localparam STAGES = 3; // log2(N)

reg signed [DATA_WIDTH-1:0] real_buf [0:N-1];
reg signed [DATA_WIDTH-1:0] imag_buf [0:N-1];

reg [2:0] sample_idx, output_idx;

reg [1:0] state;
localparam IDLE = 2'd0, LOAD = 2'd1, COMPUTE = 2'd2, OUTPUT = 2'd3;

reg signed [TWID_WIDTH-1:0] wr, wi;
wire signed [TWID_WIDTH-1:0] cos_val, sin_val;

// ROM instance
twiddle_rom rom (
    .index(output_idx[2:0]),
    .cos_val(cos_val),
    .sin_val(sin_val)
);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    state <= IDLE;
    sample_idx <= 0;
    output_idx <= 0;
    din_ready <= 0;
    dout_valid <= 0;
    done <= 0;
  end else begin
    case (state)
      IDLE: begin
        if (start) begin
          state <= LOAD;
          sample_idx <= 0;
          din_ready <= 1;
        end
      end

      LOAD: begin
        if (din_valid) begin
          real_buf[sample_idx] <= din_real;
          imag_buf[sample_idx] <= din_imag;
          sample_idx <= sample_idx + 1;
          if (sample_idx == N-1) begin
            state <= COMPUTE;
            din_ready <= 0;
          end
        end
      end

      COMPUTE: begin
        // Dummy butterfly for layout purposes
        integer i;
        for (i = 0; i < N; i = i + 1) begin
          real_buf[i] <= real_buf[i]; // placeholder to trigger synthesis
          imag_buf[i] <= imag_buf[i];
        end
        state <= OUTPUT;
        output_idx <= 0;
        dout_valid <= 1;
      end

      OUTPUT: begin
        dout_real <= real_buf[output_idx];
        dout_imag <= imag_buf[output_idx];
        output_idx <= output_idx + 1;
        if (output_idx == N-1) begin
          dout_valid <= 0;
          done <= 1;
          state <= IDLE;
        end
      end
    endcase
  end
end

endmodule
