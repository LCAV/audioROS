import rclpy
from rclpy.node import Node
import serial

from build.audio_interfaces.rosidl_generator_py.audio_interfaces.msg._signals_freq import SignalsFreq


class Gateway(Node) :
    def __init__(self):
        super().__init__("gateway",
                         automatically_declare_parameters_from_overrides=True, allow_undeclared_parameters=True)
        self.start_time = time.time()

        self.publisher_signals = self.create_publisher(
            SignalsFreq, "audio/signals_f", 10
        )

        #need the reader from the epuck initalized here

        self.desired_rate = 1000  # Hz
        self.create_timer(1 / self.desired_rate, self.publish_current_data)


    def publish_current_data(self):
        #only the first one is for my publisher
        if not self.reader_crtp.audio_dict["published"]:
            self.publish_audio_dict()
            self.reader_crtp.audio_dict["published"] = True


    def publish_audio_dict(self):
        #reader_crtp is the bluetooth reader for the crazyflie
        # read audio

        #TODO : figure out how to read the audio data from my protocol, and then that data should go in there
        #the format is all the interleaved real values of all four microphones and then all the complex values of all the micro
        #needs to replace self.reader_crtp
        signals_f_vect = self.reader_crtp.audio_dict["signals_f_vect"]
        if signals_f_vect is None:
            self.get_logger().warn("Empty audio. Not publishing")
            return

        # read frequencies
        fbins = self.reader_crtp.audio_dict["fbins"]
        if fbins is None:
            self.get_logger().warn("No data yet. Not publishing")
            return

        all_frequencies = np.fft.rfftfreq(n=N_BUFFER, d=1/FS)
        n_frequencies = len(fbins)

        # the only allowed duplicates are 0
        #if len(set(fbins[fbins>0])) < len(fbins[fbins>0]):
            #self.get_logger().warn(f"Duplicate values in fbins! unique values:{len(set(fbins))}")
            #return
        if not np.any(fbins > 0):
            self.get_logger().warn(f"Empty fbins!")
            return
        elif np.any(fbins >= len(all_frequencies)):
            self.get_logger().warn(f"Invalid fbins! {fbins[fbins > len(all_frequencies)]}")
            return

        frequencies = all_frequencies[fbins]

        #undo the interleaving and create separate matrixes for each microphone
        signals_f = np.zeros((N_MICS, n_frequencies), dtype=np.complex128)
        for i in range(N_MICS):
            signals_f[i].real = signals_f_vect[i :: N_MICS * 2]
            signals_f[i].imag = signals_f_vect[i + N_MICS :: N_MICS * 2]

        abs_signals_f = np.abs(signals_f)
        if np.any(abs_signals_f > N_BUFFER):
            self.get_logger().warn("Possibly in valid audio:")
            self.get_logger().warn(f"mic 0 {abs_signals_f[0, :5]}")
            self.get_logger().warn(f"mic 1 {abs_signals_f[1, :5]}")

        mic_positions_arr = np.array(MIC_POSITIONS)
        msg = create_signals_freq_message(signals_f.T, frequencies, mic_positions_arr,
                self.reader_crtp.audio_dict["timestamp"], self.reader_crtp.audio_dict["audio_timestamp"], FS)
        self.publisher_signals.publish(msg)

        self.get_logger().info(f"{msg.timestamp}: Published audio data with fbins {fbins[[0, 1, 2, -1]]} and timestamp {msg.audio_timestamp}")

