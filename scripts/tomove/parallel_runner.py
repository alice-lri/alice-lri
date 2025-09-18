from pathlib import Path

import os
import glob
import multiprocessing
import random
import json
import sys
import subprocess

base_folder = "../../Datasets/LiDAR"
executable = "cmake-build-release/examples/examples"

def run_experiment(experiment):
    path = experiment['path']
    output_root_folder = experiment['output_root_folder']
    accurate_digit = experiment['accurate_digit']

    # Create the output directory
    os.makedirs(output_root_folder, exist_ok=True)

    experiment_command = [executable, path, str(accurate_digit), output_root_folder]
    print(f"Executing command: {experiment_command}")

    try:
        subprocess.run(
            experiment_command,
            check=True
        )
        print(f"Process finished successfully for command: {experiment_command}")
    except subprocess.CalledProcessError as e:
        Path(output_root_folder).joinpath('error').touch()
        print(f"Error running command: {experiment_command}")
        print(f"Return Code: {e.returncode}")

def get_kitti_files():
    kitti_pattern = f'{base_folder}/kitti_organized/Organized/*/*/data/*.bin'
    kitti_files = glob.glob(kitti_pattern)

    return kitti_files

def get_durlar_files():
    durlar_pattern = f'{base_folder}/durlar/dataset/DurLAR/*/ouster_points/data/*.bin'
    durlar_files = glob.glob(durlar_pattern)

    return durlar_files

def get_conflicting_durlar_files():
    return [
        f'{base_folder}/durlar/dataset/DurLAR/DurLAR_20210716/ouster_points/data/0000035000.bin',
        f'{base_folder}/durlar/dataset/DurLAR/DurLAR_20210716/ouster_points/data/0000035800.bin',
        f'{base_folder}/durlar/dataset/DurLAR/DurLAR_20210901/ouster_points/data/0000019700.bin',
        f'{base_folder}/durlar/dataset/DurLAR/DurLAR_20211012/ouster_points/data/0000018200.bin',
        f'{base_folder}/durlar/dataset/DurLAR/DurLAR_20211012/ouster_points/data/0000019800.bin',
        f'{base_folder}/durlar/dataset/DurLAR/DurLAR_20211012/ouster_points/data/0000022800.bin',
        f'{base_folder}/durlar/dataset/DurLAR/DurLAR_20211012/ouster_points/data/0000023900.bin',
        f'{base_folder}/durlar/dataset/DurLAR/DurLAR_20211209/ouster_points/data/0000018100.bin',
        f'{base_folder}/durlar/dataset/DurLAR/DurLAR_20211209/ouster_points/data/0000021200.bin',
        f'{base_folder}/durlar/dataset/DurLAR/DurLAR_20211208/ouster_points/data/0000016700.bin',
        f'{base_folder}/durlar/dataset/DurLAR/DurLAR_20211209/ouster_points/data/0000015800.bin'
    ]

def update_progress(current, total):
    """Updates progress on the same line."""
    progress = (current / total) * 100
    sys.stdout.write(f"\rProgress: {progress:.2f}%")
    sys.stdout.flush()

def categorize_experiment(experiment):
    """Helper to categorize an experiment as successful, partially successful, or to run."""
    output_root_folder = experiment['output_root_folder']
    summary_path = f"{output_root_folder}/summary.json"
    if not os.path.exists(summary_path):
        return 'to_run'
    try:
        with open(summary_path, 'r') as f:
            json.load(f)
        return 'successful'
    except:
        return 'partially_successful'

def filter_out_successful_experiments(all_experiments, print_info=False, only_partial=False, update_interval=1000):
    successful_experiments = []
    partially_successful_experiments = []
    experiments_to_run = []

    if print_info:
        print("Filtering out successful experiments...")

    total_experiments = len(all_experiments)
    for idx, experiment in enumerate(all_experiments):
        result = categorize_experiment(experiment)
        if result == 'successful':
            successful_experiments.append(experiment)
        elif result == 'partially_successful':
            experiments_to_run.append(experiment)  # Needs to be rerun
            partially_successful_experiments.append(experiment)
        elif result == 'to_run' and not only_partial:
            experiments_to_run.append(experiment)

        # Update progress only at specified intervals
        if (idx + 1) % update_interval == 0 or idx + 1 == total_experiments:
            update_progress(idx + 1, total_experiments)

    sys.stdout.write("\n")  # Move to the next line after progress
    if print_info:
        print(f"Number of experiments: {total_experiments}")
        print(f"Number of successful experiments: {len(successful_experiments)}")
        print(f"Number of partially successful experiments: {len(partially_successful_experiments)}")
        print("")
        print(f"Number of experiments to run: {len(experiments_to_run)}")

    return experiments_to_run


def execute_experiments_parallel():
    paths_kitti = get_kitti_files()
    #paths_kitti = []
    paths_durlar = get_durlar_files()
    #paths_durlar = []
    paths = paths_kitti + paths_durlar

    random.shuffle(paths)

    print(f"Number of files: {len(paths)}")
    print(f"Number of kitti files: {len(paths_kitti)}")
    print(f"Number of durlar files: {len(paths_durlar)}")
    print("")

    accurate_digits = [4, 6, None]
    use_whole_path_for_experiment_name = True

    experiments = []
    for path in paths:
        for accurate_digit in accurate_digits:
            if 'kitti' in path and accurate_digit is not None:
                continue

            experiment_name = ''
            if 'kitti' in path:
                experiment_name += 'kitti_'
            elif 'durlar' in path and accurate_digit is not None:
                experiment_name += f'durlar_{accurate_digit}d_'
            elif 'durlar' in path:
                experiment_name += 'durlar_'


            dataset_name = 'kitti' if 'kitti' in path else 'durlar'
            if use_whole_path_for_experiment_name:
                experiment_name += f'_{path.replace("/", "_")}'

            output_root_folder = f'output/all_json/{dataset_name}/{experiment_name}'

            experiment = {
                'path': path,
                'output_root_folder': output_root_folder,
                'accurate_digit': accurate_digit,
                'experiment_name': experiment_name
            }
            experiments.append(experiment)

    experiments = filter_out_successful_experiments(experiments, print_info=True)

    N = 16# multiprocessing.cpu_count()
    print(f"Will use {N} processes for parallel execution.")
    print("Continue with the execution? (y/n): ")

    if input().lower() != 'y':
        return

    with multiprocessing.Pool(N) as pool:
        pool.map(run_experiment, experiments)


if __name__ == '__main__':
    execute_experiments_parallel()
