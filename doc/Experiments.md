# Wall Experiment Preparation

These are the steps to prepare and run a new experiment:

1. create new folder `experiments/<exp_name>` with new params.py file and PROTOCOL.md (can copy from experiments/PROTOCOL_TEMPLATE.md).
2. add <exp_name> as EXP_DIR to `src/audio_bringup/audio_bringup/measurement_pipeline.py`
3. run measurement_pipeline file

# Wall Experiment Evaluation


## Stepper motors

The following steps need to be performed in sequence to evaluate a new stepper motor dataset

1. add new dataset to `dataset_parameters.py`
2. run wall_analysis.py after adding the new dataset in parse_experiments.
3. run generate_df_results.py after adding the new dataset in the list. 
