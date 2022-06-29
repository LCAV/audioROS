# Generate pandas dataframes of results for faster processing. 

EXPERIMENT_ROOT=../datasets

.PHONY: stepper_results

all: crazyflie_demo crazyflie_flying crazyflie_stepper epuck_results

crazyflie_demo:
	python csv_or_wav_parser.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_11_23_demo --demo
	python csv_or_wav_parser.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2022_01_27_demo --demo
	python generate_flying_results.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2022_01_27_demo

crazyflie_flying:
	python csv_or_wav_parser.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_10_12_flying
	python generate_flying_results.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_10_12_flying

crazyflie_stepper:
	python csv_or_wav_parser.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_07_08_stepper_fast
	python generate_stepper_results.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_07_08_stepper_fast
	python generate_flying_results.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_07_08_stepper_fast
	python generate_filtering_results.py --platform crazyflie

epuck_results:
	python csv_or_wav_parser.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_07_27_epuck_wall --platform epuck
	python generate_stepper_results.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_07_27_epuck_wall --platform epuck
	python generate_flying_results.py --experiment_root $(EXPERIMENT_ROOT) --experiment_names 2021_07_27_epuck_wall --platform epuck
	python generate_filtering_results.py --platform epuck

clean:
	rm $(EXPERIMENT_ROOT)/2021_07_08_stepper_fast/all_data.pkl
	rm $(EXPERIMENT_ROOT)/2021_07_08_stepper_fast/backup_*.pkl
	rm $(EXPERIMENT_ROOT)/2021_07_27_epuck_wall/all_data.pkl
	rm $(EXPERIMENT_ROOT)/2021_07_27_epuck_wall/backup_*.pkl
