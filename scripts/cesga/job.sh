#!/bin/bash
#SBATCH -J accurate_ri
#SBATCH -o logs/%j.log
#SBATCH -e logs/%j.log
#SBATCH -n 64
#SBATCH -c 1
#SBATCH -t 06:00:00
#SBATCH --mem-per-cpu=3G
#SBATCH --mail-type=begin
#SBATCH --mail-type=end
#SBATCH --mail-user=s.soutullo@usc.es
set -eo pipefail

CONDA_ENV_NAME=$1
EXECUTABLE_PATH=$2
DB_DIR=$3
JOB_INDEX=$4
JOB_COUNT=$5

echo "Beginning job ${JOB_INDEX}..."

module load cesga/system miniconda3/22.11.1-1
conda activate "${CONDA_ENV_NAME}"

srun task.sh "$CONDA_ENV_NAME" "$EXECUTABLE_PATH" "$DB_DIR" "$JOB_INDEX" "$JOB_COUNT"

echo "Job ${JOB_INDEX} finished."
touch "${DB_DIR}/job_${JOB_INDEX}.success"