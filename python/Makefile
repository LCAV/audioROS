# Generate pandas dataframes of results for faster processing. 

EXPERIMENT_ROOT=../experiments

.PHONY: stepper_results

all: stepper_results epuck_results

crazyflie_demo:
	python csv_or_wav_parser.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_11_23_demo --demo

crazyflie_flying:
	python csv_or_wav_parser.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_10_12_flying
	python csv_or_wav_parser.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_11_23_demo

crazyflie_stepper_paper:
	python csv_or_wav_parser.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_10_07_stepper
	python generate_stepper_results.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_10_07_stepper

crazyflie_stepper_thesis:
	python csv_or_wav_parser.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_07_08_stepper_fast
	python generate_stepper_results.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_07_08_stepper_fast

epuck_results:
	python csv_or_wav_parser.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_07_27_epuck_wall --platform epuck
	python generate_stepper_results.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_07_27_epuck_wall --platform epuck

clean:
	rm $(EXPERIMENT_ROOT)/2021_07_08_stepper_fast/all_data.pkl
	rm $(EXPERIMENT_ROOT)/2021_07_08_stepper_fast/backup_*.pkl
	rm $(EXPERIMENT_ROOT)/2021_07_27_epuck_wall/all_data.pkl
	rm $(EXPERIMENT_ROOT)/2021_07_27_epuck_wall/backup_*.pkl
