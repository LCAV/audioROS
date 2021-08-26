import sys
import os
import argparse

def main(input_dir, output_dir):

	args = ""

	if output_dir != "":
		args = f"{args} -o {output_dir}"
	else:
		args = f"{args} -ip True"

	print(args)

	for root, dirs, files in os.walk(input_dir):
		for file in files:
			if file.endswith(".m4v") or file.endswith(".mp4"):
				target_video = os.path.join(root, file)
				print(f"Treating: {target_video}")
				os.system(f"python3 ghost_image.py -i {target_video} {args}")





if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("-i", "--input", help="path to the input file tree", required=True)
	parser.add_argument("-o", "--output", help="output directory", required=False, default = "")
	args = vars(parser.parse_args())

	main = main(args["input"], args["output"])
	#main.run()
