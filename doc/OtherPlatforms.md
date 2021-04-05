# How to add new robotic platforms to audioROS

Because of the modular design, it is straight forward to integrate new robotic platforms to the audioROS framework. 
The main requirements to add a different platform are 
- Sound can be recorded on the platform using one or more microphones.
- The platform is equipped with an IMU or more sophisticated motion tracking system to give relative movement estimates.
- The audio data can be sent to the laptop in real-time via some wireless communication protocol. If the bandwidth is too small to send the full audio signals in real time, an FFT can be performed onboard per mic and only a small number of frequency bins can be sent (more details [below](#hardware-programming))

Depending on the use case, additional requirements are needed:
- For DOA, the microphone measurements should be accompanied by an exact (ideally micro-second resolution) timestamp. 
- For wall detection, a small speaker or buzzer that can produce single-frequency sounds should be included.

The full processing pipeline for the Crazyflie drone, for the example of wall detection, is shown in the figure below.  The ROS components that need to be reimplemented for each new platform are marked in red. The top row shows the hardware pipeline specific to the Crazyflie platform, which is to be replaced with the desired robotic platform. Note that the same parts need to be replaced for the DOA use case, as for DOA vs. wall detection only the processing pipeline (after the ROS message conversion) changes.  

![Overview of the full audioROS pipeline](overview.jpg)

## ROS programming

As shown in the Figure above, the input to the audio-based algorithms are the `audio/signals_f` and `geometry/pose_raw` messages. The user thus needs to create one or multiple ROS nodes which convert the audio and motion data received from the robotic platform to these custom ROS messages. 

In the current implementation, this is done through the [`crazyflie_crtp/gateway`](https://github.com/LCAV/audioROS/blob/master/src/crazyflie_crtp/crazyflie_crtp/gateway.py) node, which fills buffers as the CRTP messages come in, and creates and publishes the ROS messages at a fixed rate. Each buffer is only sent once (thanks to a software flag) and the publishing rate is set very high to minimize the waiting time between filling the buffer and publishing the corresponding ROS message. 
The motion data is received through the Crazyflie's logging framework, which also works through the CRTP protocol. Data is received at a fixed rate, and each time a new message arrives, it is immedialtely converted to a ROS message. 

Note that you might also need to create a new [`crazyflie_description/parameters`](https://github.com/LCAV/audioROS/blob/master/src/crazyflie_description/crazyflie_description_py/parameters.py) file according to your robotic platform, containing the correct geometry of the microphones, buzzers, etc. 

## Hardware programming

It is hard to give general instructions for the hardware part as the robotic platforms may differ a lot in terms of their implementation. Below are a few recommendations based on our own experience. 

- If you are using the Crazyflie drone with your own microphone deck: rather than implementing your own pipeline for CRTP to ROS conversion, you can reuse the [ReaderCRTP](https://github.com/LCAV/crazyflie-audio/blob/master/python/reader_crtp.py) class, as long as you adjust the parameters for the number of frequency bins sent, number of mics, etc.

- If you are using a different robotic platform, you can either send the raw audio data, in which case you want to run the `processor` node to perform the FFT, or you perform the FFT onboard and send only a certain number of frequency bins. We only have experience with the latter option, so this is the recommended one. It also puts a lower load on the communication and is less prone to package loss.

## Known limitations

- The format of the buzzer commands in [`crazyflie_description/parameters`](https://github.com/LCAV/audioROS/blob/master/src/crazyflie_description/crazyflie_description_py/parameters.py) is currently quite restrictive and might not apply for different robotic platforms. 
- The `crazyflie_control/path_planning` node is not implemented yet.  

## Questions and comments

Please let us know if you are implementing the above on your own robotic platform, we are happy to give some advice! Once you have successfully integrated your own platform, feel free to add your code to the repo through a pull request. 
