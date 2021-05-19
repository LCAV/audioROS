# Sampling frequency in Hz
uint16 fs

# ROS timestamp in miliseconds
uint32 timestamp

# Number of microphones
uint8 n_mics

# Number of samples
uint16 n_buffer

# Vectorised form of signals matrix (n_mics x n_buffer)
float64[] signals_vect

# Microphone positions (n_mics x dimension)
float32[] mic_positions
