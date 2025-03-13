#!/bin/bash
#SBATCH -J accurate_ri
#SBATCH -o logs/%j.out
#SBATCH -e logs/%j.err
#SBATCH -n 1
#SBATCH -c 1
#SBATCH -t 06:00:00
#SBATCH --array=0-2047
#SBATCH --mem-per-cpu=1G
#SBATCH --mail-type=begin
#SBATCH --mail-type=end
#SBATCH --mail-user=s.soutullo@usc.es

echo "Beginning job..."

CONDA_ENV_NAME=$1
EXECUTABLE_PATH=$2
DB_DIR=$3

module load cesga/system miniconda3/22.11.1-1
conda activate "${CONDA_ENV_NAME}"

srun --export=ALL task.sh "$CONDA_ENV_NAME" "$EXECUTABLE_PATH" "$DB_DIR" "$SLURM_ARRAY_TASK_ID" "$SLURM_ARRAY_TASK_COUNT"