uint16 fs
uint32 timestamp
uint8 n_mics
uint16 n_frequencies

# List of frequencies, in Hz
uint16[] frequencies

# Vectorised form of signals matrix (n_mics x n_frequencies)
float32[] signals_real_vect
float32[] signals_imag_vect

# Microphone positions (n_mics x dimension)
float32[] mic_positions
