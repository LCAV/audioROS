# Motion logging

Tests have shown that when the flowdeck is attached to the Crazyflie and
it is moved around (without flying), then

yaw:
- stabilizer.yaw, controller.yaw and stateEstimate.yaw are all the same.
  They are in degrees and clipped to -180, 180.
- mag.x sometimes gives values similar to above 3,
  but sometimes it is constantly zero (should it only be used outside?
- gyro.z gives the raw yaw rate (or acceleration?).

x/y:
 
- motion.deltaX and motion.deltaY are in pixels and can be very noisy, especially when
  the ground is not textured. motion.deltaY needs to be inverted.
- stateEstimate.vx and stateEstimate.vy are in m/s and more stable
 but would need to be integrated for a position estimate. Note that they use a different
 reference frame, so they are inverted in the logger.
- kalman.statePX and kalman_states.vx are in m/s and almost the same as stateEstimate.vx
- kalman.stateX and stateEstimate.x are in m and almost the same. They drift a lot without flow deck.

z:
- range.zrange gives the raw data in milimeters (uint16), which is quite accurate.
- stateEstimateZ.z in milimeters (uint16), is more smooth but also overshoots a little bit, and even
  goes negative sometimes which is impossible.
- stateEstimate.z in meters (float), from barometer. Surprisingly accurate.
