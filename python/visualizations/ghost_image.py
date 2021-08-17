"""
Code from: 

https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/

"""
import cv2
import argparse
import numpy as np
from get_background import get_background, extract_roi
import signal

from constants import THRESHOLD, RADIUS, SAVE_EVERY_K, CONSECUTIVE_FRAMES

DEBUG = 0

class main:
	def __init__(self, input_file):
		self.stop_by_signal = False
		self.input_file = input_file
		self.output_file = f"outputs/{input_file.split('/')[-1].split('.')[0]}"
		return

	def signal_term_handler(self, signal, frame):
		self.stop_by_signal = True
		return

	def debug_image(self, title , image):
		cv2.imshow(title, image)
		cv2.waitKey(0) 
		cv2.destroyAllWindows() 

	def run(self):
		signal.signal(signal.SIGTERM, self.signal_term_handler)

		self.cap = cv2.VideoCapture(self.input_file)
		self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
		self.count = self.cap.get(cv2.CAP_PROP_FRAME_COUNT)
		pos = self.cap.get(cv2.CAP_PROP_POS_FRAMES)
		print("counts", self.count, pos)

		# get the background model
		print("getting background...", end="")
		background = get_background(self.cap)
		print("...done")

		# copy backroung to insert drone position
		final_patchwork = background.copy()

		background = cv2.cvtColor(background, cv2.COLOR_BGR2GRAY)


		# get the video frame height and width
		frame_width = background.shape[1]
		frame_height = background.shape[0]

		# define codec and create VideoWriter object
		#self.out = cv2.VideoWriter( self.output_file + ".mp4", cv2.VideoWriter_fourcc(*"mp4v"), 10, (frame_width, frame_height), )

		frame_counter = 0

		consecutive_frame = CONSECUTIVE_FRAMES

		frame_list = []

		while self.cap.isOpened():

			if self.stop_by_signal is True:
				print("Want to stop...")
				raise KeyboardInterrupt

			ret, frame = self.cap.read()

			if ret is True:
				frame_counter += 1


				# append the final result into the `frame_list`
				# if we have reached `consecutive_frame` number of frames
				if (frame_counter % consecutive_frame) == 0:

					frame = extract_roi(frame)
					print(f"treating frame {frame_counter}...", end="")
					#self.debug_image("original frame", frame)

					last_annotated = None

					gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

					# find the difference between current frame and base frame
					frame_diff = cv2.absdiff(gray, background)

#					# threshold for noise reduction
					frame_diff[frame_diff < 50] = 0

					# thresholding to convert the frame to binary
					ret, frame_diff = cv2.threshold(frame_diff, 40, 255, cv2.THRESH_BINARY)

					# dilatatation to enclose the whole drone without separate blobs
					dilated_frame = cv2.dilate(frame_diff, None, iterations=7)

					if DEBUG:
						self.debug_image("dilated_frame", dilated_frame)

					# find the contours around the white segmented areas
					contours, hierarchy = cv2.findContours( dilated_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

					if (int(cv2.__version__[0]) > 3):
						contours, hierarchy = cv2.findContours(dilated_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
					else:
						im2, contours, hierarchy = cv2.findContours(dilated_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

					contour_max = max(contours, key = cv2.contourArea)

					im_contour = frame.copy()
					
					# continue through the loop if contour area is too small...
					# ... helps in removing noise detection
					area = cv2.contourArea(contour_max)

					print(f"contour area is {area}")
					
					if DEBUG:
						# debug print
						cv2.drawContours(im_contour, contour_max, -1, (0, 255, 0), 3)
						
					if area < 7000:
						print("Contour not used, too small")
						continue

					if area > 35000:
						print("Contour not used, too big")
						continue

					# find enclosing circle
					circle_center, rad = cv2.minEnclosingCircle(contour_max)
					circle_center = (int(circle_center[0]), int(circle_center[1]))
					
					if DEBUG:
						print(f"enclosing diameter {rad}")

					if rad > 200:
						print("Contour not used, too large enclosing diameter")
						continue

					if DEBUG:
						#plot for debug
						cv2.circle( im_contour, circle_center, int(rad), (0, 0, 255), thickness=-1, )
						self.debug_image("im_contour", im_contour)
					
					#creating circle mask arround the area of interrest
					circle_mask = np.zeros_like(frame_diff)
					cv2.circle(circle_mask, circle_center, int(rad), 255, -1)

					ajusted_mask = dilated_frame.copy()
					ajusted_mask[circle_mask == 0] = 0
					
					if DEBUG:
						self.debug_image("ajusted_mask", ajusted_mask)

					final_patchwork[ajusted_mask > 0, :] = frame[ajusted_mask > 0 , :]

					if DEBUG:
						self.debug_image("final_patchwork", final_patchwork)

					frame_list.append(frame_diff)

					if cv2.waitKey(100) & 0xFF == ord("q"):
						print("cv2 exit detected")
						break

				# if self.count == frame_counter:
				# #if frame_counter > 400:

				# 	self.debug_image("final_patchwork", final_patchwork)
				# 	cv2.imwrite(self.output_file + "_final.png", final_patchwork)
				# print("...done")
			else:
				self.debug_image("final_patchwork", final_patchwork)
				print("end of stream")				
				print("Writing to output file: ", end = "")
				print(self.output_file + "_final.png")
				cv2.imwrite(self.output_file + "_final.png", final_patchwork)
				break
		#self.out.release()
		self.stop()

	def stop(self):
		print("stopping main...")
		self.cap.release()
		cv2.destroyAllWindows()


if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("-i", "--input", help="path to the input video", required=True)
	args = vars(parser.parse_args())
	#try:
	main = main(args["input"])
	main.run()

	# except Exception as e:
		# print(e)
		# main.cap.release()
		# cv2.destroyAllWindows()
		# print("all released after Exception.")
