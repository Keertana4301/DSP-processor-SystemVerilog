# DSP-processor-SystemVerilog

A configurable digital signal processing (DSP) module implemented in SystemVerilog, supporting three processing modes: FIR filtering, IIR filtering, and FFT computation. This design is suitable for FPGA implementation and includes comprehensive testbench verification.

## Features

- **Multi-mode operation**: FIR, IIR, and FFT processing in a single module
- **Streaming interface**: AXI4-Stream compatible input/output
- **Configurable parameters**: Adjustable data width, coefficient width, and processing sizes
- **Real-time coefficient loading**: Dynamic coefficient updates without system reset
- **Pipeline-friendly design**: Optimized for high-throughput applications

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    DSP_TOP Module                       │
├─────────────────────────────────────────────────────────┤
│  Mode Selection (2-bit)                                 │
│  ┌─────────┐    ┌─────────┐    ┌─────────────────┐      │
│  │   FIR   │    │   IIR   │    │   FFT_RADIX2    │      │
│  │ Filter  │    │ Filter  │    │   Transform     │      │
│  └─────────┘    └─────────┘    └─────────────────┘      │
└─────────────────────────────────────────────────────────┘
```

## Module Specifications

### DSP_TOP Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| DATA_WIDTH | 16 | Input/output data width in bits |
| COEF_WIDTH | 16 | Coefficient width in bits |
| MAX_TAPS | 8 | Maximum FIR filter taps |
| IIR_SECTIONS | 2 | Number of IIR biquad sections |
| FFT_N | 8 | FFT size (power of 2) |

### Operating Modes
- **Mode 00**: FIR Filter - Finite Impulse Response filtering
- **Mode 01**: IIR Filter - Infinite Impulse Response filtering (cascaded biquads)
- **Mode 10**: FFT - Fast Fourier Transform (radix-2 decimation-in-time)

## Interface Description

### Input/Output Ports

#### Clock and Reset
```systemverilog
input  logic clk,           // System clock
input  logic rst_n,         // Active-low reset
```

#### Mode Control
```systemverilog
input  logic [1:0] mode,    // 00=FIR, 01=IIR, 10=FFT
input  logic start,         // Start processing (FFT mode)
output logic done,          // Processing complete (FFT mode)
```

#### Data Streaming Interface
```systemverilog
// Input stream
input  logic signed [DATA_WIDTH-1:0] din_real,
input  logic signed [DATA_WIDTH-1:0] din_imag,
input  logic din_valid,
output logic din_ready,

// Output stream  
output logic signed [DATA_WIDTH-1:0] dout_real,
output logic signed [DATA_WIDTH-1:0] dout_imag,
output logic dout_valid,
```

#### Coefficient Interfaces
```systemverilog
// FIR coefficients
input  logic fir_coeff_wr_en,
input  logic [$clog2(MAX_TAPS)-1:0] fir_coeff_index,
input  logic signed [COEF_WIDTH-1:0] fir_coeff_value,

// IIR coefficients  
input  logic iir_coeff_wr_en,
input  logic [$clog2(IIR_SECTIONS)-1:0] iir_section_index,
input  logic [2:0] iir_coeff_sel,  // 0=b0, 1=b1, 2=b2, 3=a1, 4=a2
input  logic signed [COEF_WIDTH-1:0] iir_coeff_value,
```

## Individual Module Details

### FIR Filter Module
- **Type**: Direct-form transposed structure
- **Taps**: Configurable up to MAX_TAPS
- **Latency**: 1 clock cycle
- **Features**: Dynamic coefficient loading, built-in shift register

### IIR Filter Module  
- **Type**: Cascaded biquad sections (Direct Form I)
- **Sections**: Configurable number of second-order sections
- **Transfer Function**: H(z) = ∏ᵢ (b₀ᵢ + b₁ᵢz⁻¹ + b₂ᵢz⁻²)/(1 + a₁ᵢz⁻¹ + a₂ᵢz⁻²)
- **Features**: Dynamic coefficient loading, internal state management

### FFT Module
- **Algorithm**: Radix-2 decimation-in-time
- **Size**: Configurable power-of-2 sizes
- **Type**: Streaming with internal buffering
- **Features**: Complex input/output, built-in twiddle factor generation

## Simulation and Testing

### Waveform

![Screenshot 2025-06-18 113321](https://github.com/user-attachments/assets/748bed92-9c61-4209-b5b3-8f9f8bcb840a)


## Known Limitations

1. **FFT Size**: Limited to powers of 2 (configurable at synthesis)
2. **IIR Sections**: Fixed number of biquad sections
3. **Coefficient Precision**: Limited by COEF_WIDTH parameter
4. **No Overflow Protection**: User must ensure proper scaling

## Future Enhancements

- [ ] Variable FFT size support
- [ ] Overflow detection and saturation
- [ ] Windowing functions for FFT
- [ ] Additional filter types (elliptic, Chebyshev)
- [ ] AXI4-Stream master/slave interfaces
- [ ] Configuration register interface

## License

This project is released under the MIT License. See LICENSE file for details.

