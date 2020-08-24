uint16 fs
uint32 timestamp
uint8 n_mics
uint16 n_buffer

# Vectorised form of signals matrix (n_mics x n_buffer)
float64[] signals_vect

# Microphone positions (n_mics x dimension)
float32[] mic_positions
