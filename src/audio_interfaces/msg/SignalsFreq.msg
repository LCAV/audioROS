# Sampling frequency in Hz
uint16 fs 32000
# ROS timestamp in miliseconds
uint32 timestamp 0
# Audio timestamp in microseconds
uint32 audio_timestamp 0
# Number of microphones
uint8 n_mics 4
# Number of frequencies
uint16 n_frequencies 2

# List of frequencies, in Hz
uint16[] frequencies [1000, 2000]

# Vectorised form of signals matrix (n_mics x n_frequencies)
float32[] signals_real_vect [1, 2, 3, 4, 1, 2, 3, 4]
float32[] signals_imag_vect [0, 1, 0, 1, 0, 1, 0, 1]

# Microphone positions (n_mics x dimension)
float32[] mic_positions [0, 1, 2, 3, 4, 5, 6, 7]
