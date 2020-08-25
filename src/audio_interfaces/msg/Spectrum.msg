uint32 timestamp
uint16 n_frequencies
uint16 n_angles

# Estimate of orientation at which this spectrum is taken
float32 orientation 0

# List of frequencies, in Hz
float32[] frequencies

# Spatial spectrum data for different frequencies (n_frequencies x n_angles)
float32[] spectrum_vect
