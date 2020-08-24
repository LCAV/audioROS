uint32 timestamp
uint8 n_mics
uint16 n_frequencies

# List of frequencies, in Hz
uint16[] frequencies

# Autocorrelation data (real and imaginary parts separated), originally of shape (n_frequencies x n_mics x n_mics)
# Note that high precision here is important, because the
# conditioning of the matrix is very sensitive to small 
# differences. 
float64[] corr_real_vect
float64[] corr_imag_vect

# Microphone positions (n_mics x dimension)
float32[] mic_positions
