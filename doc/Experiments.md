# Wall Experiment 

## Preparation

These are the steps to prepare and run a new experiment:

1. create new folder `experiments/<exp_name>` with new params.py file and PROTOCOL.md (can copy from experiments/PROTOCOL_TEMPLATE.md).
2. add <exp_name> as EXP_DIR to `src/audio_bringup/audio_bringup/measurement_pipeline.py`
3. run measurement_pipeline file

## Stepper motors evaluation

The following steps need to be performed in sequence to evaluate a new stepper motor dataset

1. run wall_analysis.py after adding the new dataset in parse_experiments. *TODO move this to dataset_parameters*
3. (optional): run CleanupAnalysis.ipynb to find good parameters for cleaning up. 
2. add new dataset to `dataset_parameters.py`
4. run generate_df_results.py after adding the new dataset in the list. *TODO move list here to dataset_parameters*
