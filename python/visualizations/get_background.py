import numpy as np
import cv2

from constants import ROI_X, ROI_Y

DEBUG = False

N_FRAMES = 20


def cancel_roi(frame):
    height, width, *_ = frame.shape
    # return frame[int(height * ROI_Y[0]) : int(height * ROI_Y[1]),int(width * ROI_X[0]) : int(width * ROI_X[1]),	:,]
    mask = np.zeros(frame.shape)
    mask[
        int(height * ROI_Y[0]) : int(height * ROI_Y[1]),
        int(width * ROI_X[0]) : int(width * ROI_X[1]),
    ] = 1
    mask_inv = np.abs(mask - 1)

    if DEBUG:
        cv2.imshow("Debug frame", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        cv2.imshow("Debug mask", mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        cv2.imshow("Debug inverted mask", mask_inv)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    frame[mask_inv > 0] = 0

    return frame


def get_background(container, verbose=True):
    # we will randomly select 50 frames for the calculating the median
    total_frames = container.streams.video[0].frames
    sampling = total_frames // N_FRAMES
    print(f"using each {sampling}th frame")

    index = 0
    frames = []
    for frame in container.decode(video=0):
        index += 1
        if index % sampling == 0:
            frames.append(frame.to_ndarray(format="rgb24"))

    # calculate the median
    print("calculate median of", len(frames))
    median_frame = np.median(frames, axis=0).astype(np.uint8)
    return median_frame
