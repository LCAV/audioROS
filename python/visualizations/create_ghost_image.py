"""
Create an image from video where moving parts are overlaid. 
"""
import argparse
import math
import os

import av
import cv2
import matplotlib.pyplot as plt
import numpy as np
import signal

from get_background import get_background, cancel_roi

DEBUG_IMAGE_FRAMES = True

# for demo
# CONSECUTIVE_FRAME = 90  # use every nth frame
# START_TIME = 16
# RADIUS_FACTOR = 1.5
# MAX_AREA = 3000
# FLIP_VERTICAL = True
# END_TIME = 170

# for flying
CONSECUTIVE_FRAME = 60  # use every nth frame
START_TIME = 8
END_TIME = 28
RADIUS_FACTOR = 1.0
MAX_AREA = 15000
FLIP_VERTICAL = False


MIN_AREA = 30

# ROTATE = 180 # rotate image


class main:
    def __init__(self, input_file, output_folder, ext, output_in_place):
        self.stop_by_signal = False
        self.input_file = input_file
        self.input_file_name = input_file.split("/")[-1].split(".")[0]
        print(f"input file :{self.input_file}")
        if output_in_place:
            self.output_file = (
                f"{os.path.dirname(input_file)}/{self.input_file_name}{ext}.png"
            )
        else:
            self.output_file = f"{output_folder}{self.input_file_name}{ext}.png"

            if not os.path.exists(output_folder):
                os.makedirs(output_folder)
                print(f"Created output folder: {output_folder}")
        print(f"output file:{self.output_file}")
        return

    def signal_term_handler(self, signal, frame):
        self.stop_by_signal = True
        return

    def debug_image(self, title, image):
        from PIL import Image

        img = Image.fromarray(image)
        if not os.path.exists("tests/"):
            os.makedirs("tests/")
        img.save(f"tests/test_{title}.jpg", quality=95)
        # cv2.imshow(title, image)
        # cv2.waitKey(0)

    def run(self):
        signal.signal(signal.SIGTERM, self.signal_term_handler)

        self.container = av.open(self.input_file)
        count = self.container.streams.video[0].frames
        frame_rate_frac = self.container.streams.video[0].average_rate
        frame_rate = frame_rate_frac.numerator / frame_rate_frac.denominator
        print("frame_rate:", frame_rate)

        #  get the background model
        background = cv2.imread(self.input_file.split(".")[0] + ".jpg")
        if background is None:
            print("getting background...")
            background = get_background(self.container)

            self.container.close()
            cv2.imwrite(
                self.input_file.split(".")[0] + ".jpg",
                cv2.cvtColor(background, cv2.COLOR_RGB2BGR),
            )
            print("...done")
        else:
            background = cv2.cvtColor(background, cv2.COLOR_BGR2RGB)

        if FLIP_VERTICAL:
            background = background[::-1, :, :]

        # copy backroung to insert drone position
        final_patchwork = background.copy()

        background = cv2.cvtColor(background, cv2.COLOR_BGR2GRAY)

        self.container = av.open(self.input_file)
        for frame_counter, frame_obj in enumerate(self.container.decode(video=0)):
            if (frame_counter % CONSECUTIVE_FRAME) != 0:
                continue
            time = frame_counter / frame_rate
            if time < START_TIME:
                continue
            elif time > END_TIME:
                break

            frame = frame_obj.to_ndarray(format="rgb24")
            if FLIP_VERTICAL:
                frame = frame[::-1, :, :]

            if self.stop_by_signal is True:
                print("Want to stop...")
                raise KeyboardInterrupt

            print(
                f"treating frame {frame_counter}/{count}, {frame_counter / frame_rate:.1f}s/{count/frame_rate:.1f}s"
            )

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # find the difference between current frame and base frame
            frame_diff = cv2.absdiff(gray, background)
            # threshold for noise reduction
            frame_diff[frame_diff < 10] = 0

            # thresholding to convert the frame to binary
            ret, frame_diff = cv2.threshold(frame_diff, 40, 255, cv2.THRESH_BINARY)

            # dilatatation to enclose the whole drone without separate blobs
            # dilated_frame = cv2.erode(frame_diff, None, iterations=2)
            dilated_frame = frame_diff
            dilated_frame = cv2.dilate(dilated_frame, None, iterations=7)
            # dilated_frame = cv2.erode(dilated_frame, None, iterations=1)

            if DEBUG_IMAGE_FRAMES:
                self.debug_image("frame_diff", frame_diff)
                self.debug_image("dilated_frame", dilated_frame)

            # find the contours around the white segmented areas
            if int(cv2.__version__[0]) > 3:
                contours, hierarchy = cv2.findContours(
                    dilated_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
            else:
                im2, contours, hierarchy = cv2.findContours(
                    dilated_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )

            if not len(contours):
                continue

            contour_max = max(contours, key=cv2.contourArea)
            for contour_max in [contour_max]:  # contours:

                im_contour = frame.copy()

                area = cv2.contourArea(contour_max)
                if DEBUG_IMAGE_FRAMES:
                    cv2.drawContours(im_contour, contour_max, -1, (0, 255, 0), 3)

                if area < MIN_AREA:
                    print(f"Contour not used, too small: {area:.0f}")
                    continue

                if area > MAX_AREA:
                    print(f"Contour not used, too big area: {area:.0f}")
                    continue

                print("contour used:", area)

                # find enclosing circle
                circle_center, radius = cv2.minEnclosingCircle(contour_max)
                circle_center = (int(circle_center[0]), int(circle_center[1]))

                radius *= RADIUS_FACTOR
                if DEBUG_IMAGE_FRAMES:
                    cv2.circle(
                        im_contour,
                        circle_center,
                        int(radius),
                        (0, 0, 255),
                        thickness=-1,
                    )
                    self.debug_image("im_contour", im_contour)

                # creating circle mask arround the area of interrest
                circle_mask = np.zeros_like(frame_diff)
                cv2.circle(circle_mask, circle_center, int(radius), 255, -1)

                ajusted_mask = dilated_frame.copy()
                ajusted_mask[circle_mask == 0] = 0

                final_patchwork[ajusted_mask > 0, :] = frame[ajusted_mask > 0, :]
                # final_patchwork[circle_mask > 0, :] = frame[circle_mask > 0, :]

        print("Writing to output file: ", self.output_file)
        cv2.imwrite(self.output_file, cv2.cvtColor(final_patchwork, cv2.COLOR_RGB2BGR))
        self.stop()

    def stop(self):
        self.container.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", help="path to the input video", required=True)
    parser.add_argument(
        "-o",
        "--output",
        help="path to the output folder ending with /",
        required=False,
        default="output/",
    )
    parser.add_argument(
        "-ext",
        "--ext",
        help="extension name for the output image",
        required=False,
        default="_merged",
    )
    parser.add_argument(
        "-ip",
        "--output_in_place",
        help="ignore the output folder and place output with input video",
        required=False,
        default=False,
    )

    args = vars(parser.parse_args())
    main = main(args["input"], args["output"], args["ext"], args["output_in_place"])
    main.run()
