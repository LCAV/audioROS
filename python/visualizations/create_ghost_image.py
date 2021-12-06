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

MARGIN_NEXT_FRAME = 1.6
DEBUG = True
DEBUG_PLOT_IMAGE = True
DEBUG_BACKGROUND = False
DEBUG_IMAGE_FRAMES = False
DEBUG_LAST = False

EPUCK = False


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
        if DEBUG_PLOT_IMAGE:
            cv2.imshow(title, image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def run(self):
        signal.signal(signal.SIGTERM, self.signal_term_handler)

        # self.cap = cv2.VideoCapture(self.input_file)
        # self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        # self.count = self.cap.get(cv2.CAP_PROP_FRAME_COUNT)
        # pos = self.cap.get(cv2.CAP_PROP_POS_FRAMES)
        self.container = av.open(self.input_file)
        self.count = self.container.streams.video[0].frames
        print("counts", self.count, 0)

        #  get the background model
        print("getting background...", end="")
        background = get_background(self.container)

        self.container.close()

        print("...done")

        # copy backroung to insert drone position
        final_patchwork = background.copy()

        background = cv2.cvtColor(background, cv2.COLOR_BGR2GRAY)

        if DEBUG_BACKGROUND:
            self.debug_image("Background image", background)

        frame_counter = 0

        consecutive_frame = 3  # CONSECUTIVE_FRAMES

        frame_list = []
        radius_vect = []
        last_snapshot_position = []

        self.container = av.open(self.input_file)
        for frame_obj in self.container.decode(video=0):

            frame = frame_obj.to_ndarray(format="rgb24")

            if self.stop_by_signal is True:
                print("Want to stop...")
                raise KeyboardInterrupt

            frame_counter += 1

            # append the final result into the `frame_list`
            # if we have reached `consecutive_frame` number of frames
            if (frame_counter % consecutive_frame) == 0:

                print(f"treating frame {frame_counter}/{self.count}...", end="")

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # find the difference between current frame and base frame
                frame_diff = cv2.absdiff(gray, background)

                # threshold for noise reduction
                frame_diff[frame_diff < 50] = 0

                # thresholding to convert the frame to binary
                ret, frame_diff = cv2.threshold(frame_diff, 40, 255, cv2.THRESH_BINARY)

                # dilatatation to enclose the whole drone without separate blobs
                if not EPUCK:
                    dilated_frame = cv2.dilate(frame_diff, None, iterations=7)
                else:
                    dilated_frame = cv2.erode(frame_diff, None, iterations=4)
                    dilated_frame = cv2.dilate(dilated_frame, None, iterations=14)

                if DEBUG_IMAGE_FRAMES:
                    self.debug_image("dilated_frame", dilated_frame)

                dilated_frame = cancel_roi(dilated_frame)

                if DEBUG_IMAGE_FRAMES:
                    self.debug_image("dilated_frame after ROI", dilated_frame)

                # find the contours around the white segmented areas
                if int(cv2.__version__[0]) > 3:
                    contours, hierarchy = cv2.findContours(
                        dilated_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                    )
                else:
                    im2, contours, hierarchy = cv2.findContours(
                        dilated_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                    )

                if contours == []:
                    continue
                else:
                    contour_max = max(contours, key=cv2.contourArea)

                im_contour = frame.copy()

                # continue through the loop if contour area is too small...
                # ... helps in removing noise detection
                area = cv2.contourArea(contour_max)

                print(f"contour area is {area}")

                if DEBUG_IMAGE_FRAMES:
                    # debug print
                    cv2.drawContours(im_contour, contour_max, -1, (0, 255, 0), 3)

                if EPUCK:
                    min_area = 500
                else:
                    min_area = 7000

                if area < min_area:
                    print("Contour not used, too small")
                    continue

                if EPUCK:
                    max_area = 30000
                else:
                    max_area = 35000

                if area > max_area:
                    print(f"Contour not used, too big area: {area}")
                    continue

                # find enclosing circle
                circle_center, radius = cv2.minEnclosingCircle(contour_max)

                # for e-puck
                if EPUCK:
                    radius = 1.3 * radius

                circle_center = (int(circle_center[0]), int(circle_center[1]))
                radius_vect.append(radius)

                if len(last_snapshot_position) >= 1:
                    dist = np.linalg.norm(
                        np.array(circle_center) - np.array(last_snapshot_position[-1])
                    )
                    if DEBUG_LAST:
                        print(
                            f"radius is : {radius}, last position {last_snapshot_position[-1]}, current position {circle_center}"
                        )
                    if dist > MARGIN_NEXT_FRAME * radius:
                        last_snapshot_position.append(circle_center)
                        print("Snapshot taken")
                    else:
                        print(
                            f"Snapshot passed, no movement big enough, distance: {dist}"
                        )
                        continue
                else:
                    last_snapshot_position.append(circle_center)

                if DEBUG_IMAGE_FRAMES:
                    print(f"enclosing diameter {radius}")

                if radius > 200:
                    print("Contour not used, too large enclosing diameter")
                    continue

                if DEBUG_IMAGE_FRAMES:
                    # plot for debug
                    cv2.circle(
                        im_contour,
                        circle_center,
                        int(radius),
                        (0, 0, 255),
                        thickness=-1,
                    )
                    self.debug_image("im_contour", im_contour)

                    print(
                        f"last_snapshot_position: {last_snapshot_position[-1]}, type {type(last_snapshot_position)}, length {len(last_snapshot_position)}"
                    )

                # creating circle mask arround the area of interrest
                circle_mask = np.zeros_like(frame_diff)
                cv2.circle(circle_mask, circle_center, int(radius), 255, -1)

                ajusted_mask = dilated_frame.copy()
                ajusted_mask[circle_mask == 0] = 0

                if EPUCK:
                    ajusted_mask = circle_mask

                if DEBUG_IMAGE_FRAMES:
                    self.debug_image("ajusted_mask", ajusted_mask)

                final_patchwork[ajusted_mask > 0, :] = frame[ajusted_mask > 0, :]

                if DEBUG_IMAGE_FRAMES:
                    self.debug_image("final_patchwork", final_patchwork)

                frame_list.append(frame_diff)

        # TODO: Plot last one
        # TODO: Lower resolution on 2021_07_14_flying_hover
        # TODO: epuck

        # print(f"data: {data[abs(data - np.mean(data)) < m * np.std(data)]}")
        self.debug_image("final_patchwork", final_patchwork)
        print("end of stream")
        print("Writing to output file: ", end="")
        print(self.output_file)
        cv2.imwrite(self.output_file, final_patchwork)

        if 0:
            fig = plt.figure()
            plt.plot(radius_vect, ".-")
            plt.title("Radius of detected drones")
            plt.show()
            print(f"radius_vect = {radius_vect}")
            print(f"std radius_vect = {np.std(radius_vect)}")
        self.stop()

    def stop(self):
        print("stopping main...")
        self.container.close()
        cv2.destroyAllWindows()


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
        default="_merged_new",
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