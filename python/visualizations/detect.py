"""
Code from: 

https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/

"""
import cv2
import argparse
from get_background import get_background, extract_roi
import signal

from constants import THRESHOLD, RADIUS, SAVE_EVERY_K, CONSECUTIVE_FRAMES


class main:
	def __init__(self, input_file):
		self.stop_by_signal = False
		self.input_file = input_file
		self.output_file = f"outputs/{input_file.split('/')[-1].split('.')[0]}"
		return

	def signal_term_handler(self, signal, frame):
		self.stop_by_signal = True
		return

	def run(self):
		signal.signal(signal.SIGTERM, self.signal_term_handler)

		self.cap = cv2.VideoCapture(self.input_file)
		self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
		count = self.cap.get(cv2.CAP_PROP_FRAME_COUNT)
		pos = self.cap.get(cv2.CAP_PROP_POS_FRAMES)
		print("counts", count, pos)

		# get the background model
		print("getting background...", end="")
		background = get_background(self.cap)
		print("...done")
		background = cv2.cvtColor(background, cv2.COLOR_BGR2GRAY)

		# get the video frame height and width
		frame_width = background.shape[1]
		frame_height = background.shape[0]

		# define codec and create VideoWriter object
		self.out = cv2.VideoWriter(
			self.output_file + ".mp4",
			cv2.VideoWriter_fourcc(*"mp4v"),
			10,
			(frame_width, frame_height),
		)

		all_points = {}
		frame_count = 0
		last_save = 0
		consecutive_frame = CONSECUTIVE_FRAMES
		while self.cap.isOpened():

			if self.stop_by_signal == True:
				print("Want to stop...")
				raise KeyboardInterrupt

			ret, frame = self.cap.read()
			if ret == True:
				frame_count += 1

				frame = extract_roi(frame)
				print(f"treating frame {frame_count}...", end="")

				orig_frame = frame.copy()
				last_annotated = None

				gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
				if frame_count % consecutive_frame == 0 or frame_count == 1:
					frame_diff_list = []

				# find the difference between current frame and base frame
				frame_diff = cv2.absdiff(gray, background)
				# thresholding to convert the frame to binary
				ret, thres = cv2.threshold(
					frame_diff, THRESHOLD, 255, cv2.THRESH_BINARY
				)

				# dilate the frame a bit to get some more white area...
				# ... makes the detection of contours a bit easier
				dilate_frame = cv2.dilate(thres, None, iterations=2)
				# append the final result into the `frame_diff_list`
				frame_diff_list.append(dilate_frame)
				# if we have reached `consecutive_frame` number of frames
				if len(frame_diff_list) == consecutive_frame:
					# add all the frames in the `frame_diff_list`
					sum_frames = sum(frame_diff_list)
					# find the contours around the white segmented areas
					image, contours, hierarchy = cv2.findContours(
						sum_frames, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
					)
					for contour in contours:
						# continue through the loop if contour area is less than 500...
						# ... helps in removing noise detection
						if cv2.contourArea(contour) < 500:
							continue

						circle_center, rad = cv2.minEnclosingCircle(contour)
						circle_center = (int(circle_center[0]), int(circle_center[1]))

						if circle_center not in all_points.keys():
							all_points[circle_center] = 0
						all_points[circle_center] += 1

						for circle_center in all_points.keys():
							cv2.circle(
								orig_frame,
								circle_center,
								int(RADIUS),
								(0, 0, 255),
								thickness=-1,
							)

					cv2.imshow("Detected Objects", orig_frame)
					self.out.write(orig_frame)
					last_annotated = orig_frame

					if (frame_count - last_save) > SAVE_EVERY_K:
						fname = self.output_file + f"_{frame_count}.png"
						cv2.imwrite(fname, orig_frame)
						last_save = frame_count
						print("wrote", fname)

					if cv2.waitKey(100) & 0xFF == ord("q"):
						print("cv2 exit detected")
						break

				if last_annotated is not None:
					cv2.imwrite(self.output_file + "_final.png", last_annotated)
				print("...done")
			else:
				break
		self.out.release()
		self.stop()

	def stop(self):
		print("stopping main...")
		self.cap.release()
		cv2.destroyAllWindows()


if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("-i", "--input", help="path to the input video", required=True)
	args = vars(parser.parse_args())
	try:
		main = main(args["input"])
		main.run()

	except Exception as e:
		print(e)
		main.cap.release()
		cv2.destroyAllWindows()
		print("all released after Exception.")
