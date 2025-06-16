module twiddle_rom #(parameter TWID_WIDTH = 16) (
    input  wire [2:0] index,
    output reg signed [TWID_WIDTH-1:0] cos_val,
    output reg signed [TWID_WIDTH-1:0] sin_val
);

always @(*) begin
    case (index)
        3'd0: begin cos_val =  16'sd32767; sin_val =  16'sd0;      end
        3'd1: begin cos_val =  16'sd23170; sin_val = -16'sd23170; end
        3'd2: begin cos_val =  16'sd0;      sin_val = -16'sd32767; end
        3'd3: begin cos_val = -16'sd23170; sin_val = -16'sd23170; end
        default: begin cos_val = 16'sd0; sin_val = 16'sd0; end
    endcase
end

endmodule
