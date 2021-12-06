import numpy as np
import cv2

from constants import ROI_X, ROI_Y

DEBUG = False
COUNT_FOR_BACKGROUND = 50


def cancel_roi(frame):
    height, width, *_ = frame.shape
    # return frame[int(height * ROI_Y[0]) : int(height * ROI_Y[1]),int(width * ROI_X[0]) : int(width * ROI_X[1]),	:,]
    print(f"height, width = {height}, {width}")

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


def get_background(container):
    # we will randomly select 50 frames for the calculating the median
    frame_number = container.streams.video[0].frames
    sampling = int(frame_number / COUNT_FOR_BACKGROUND / 2)
    print(
        f"number of frame: {container.streams.video[0].frames}/{COUNT_FOR_BACKGROUND} = {sampling}",
        end="",
    )

    index = 0

    frames = []
    for frame in container.decode(video=0):
        index += 1

        if index % sampling == 0:
            # frame = extract_roi(frame)
            frames.append(frame.to_ndarray(format="rgb24"))

    # calculate the median
    median_frame = np.median(frames, axis=0).astype(np.uint8)
    return median_frame
