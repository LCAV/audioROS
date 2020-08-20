uint32 timestamp
uint16 fs
uint8 n_mics
uint16 n_frequencies

# List of frequencies, in Hz
uint16[] frequencies

# Vectorised form of signals matrix, which 
# is of shape n_mics x n_frequencies
float32[] signals_real_vect
float32[] signals_imag_vect
