uint32 timestamp
uint8 n_mics
uint8 n_frequencies

# List of frequencies, in Hz
float32[] frequencies

# Autocorrelation data (real and imaginary parts separated)
# note that high precision here is important, because the
# conditioning of the matrix is very sensitive to small 
# differences. 
float64[] real_vect
float64[] imag_vect
