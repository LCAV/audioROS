# Explanation of motion loggers 

 Tests have shown that when the flowdeck is attached to the Crazyflie and
 it is moved around (without flying), then
 
 yaw:
 - stabilizer.yaw, controller.yaw and stateEstimate.yaw are all the same.
   They are in degrees and clipped to -180, 180.
 - mag.x sometimes gives values similar to above 3,
   but sometimes it is constantly zero (should it only be used outside?
 - gyro.z gives the raw yaw rate (or acceleration?).
 
 dx/dy:
 - motion.deltaX and motion.deltaY are in milimeters can be very noisy, especially when
   the ground is not textured. motion.deltaY points in "wrong" direction.
 - stateEstimateZ.vx and stateEstimateZ.vy are in mm/s and more stable
  but of course would need to be integrated for a position estimate.
 - stateEstimate.vx and stateEstimate.vy are in m/s and more stable
  but of course would need to be integrated for a position estimate. Note that they use a different
  reference frame than motion.deltaX, so they are inverted in the logger.
 - kalman.PX is in m/s and almost the same as stateEstimate.vx
 - kalman.X is in m and quite a smooth position estimate

z:
 - range.zrange gives the raw data in milimeters (uint16), which is quite accurate.
 - stateEstimateZ.z in milimeters (uint16), is more smooth but also overshoots a little bit, and even
   goes negative sometimes which is impossible.
 - stateEstimate.z in meters (float), from barometer. Surprisingly accurate.
