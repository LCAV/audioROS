import numpy as np
import cv2

from constants import ROI_X, ROI_Y, N_IMAGES_BACKGROUND


def extract_roi(frame):
    height, width, *_ = frame.shape
    return frame[
        int(height * ROI_Y[0]) : int(height * ROI_Y[1]),
        int(width * ROI_X[0]) : int(width * ROI_X[1]),
        :,
    ]


def get_background(cap):
    # we will randomly select 50 frames for the calculating the median
    frame_indices = cap.get(cv2.CAP_PROP_FRAME_COUNT) * np.random.uniform(
        size=N_IMAGES_BACKGROUND
    )
    # we will store the frames in array
    frames = []
    for idx in frame_indices:
        # set the frame id to read that particular frame
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ret, frame = cap.read()

        frame = extract_roi(frame)
        frames.append(frame)
    # calculate the median
    median_frame = np.median(frames, axis=0).astype(np.uint8)
    return median_frame
