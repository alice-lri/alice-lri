#!/bin/bash
#SBATCH -J accurate_ri
#SBATCH -o logs/%j.out
#SBATCH -e logs/%j.err
#SBATCH -n 2048
#SBATCH -c 1
#SBATCH -t 06:00:00
#SBATCH --mem-per-cpu=1G
#SBATCH --mail-type=begin
#SBATCH --mail-type=end
#SBATCH --mail-user=s.soutullo@usc.es

echo "Beginning job..."

export CONDA_ENV_NAME=$1
export EXECUTABLE_PATH=$2
export DB_DIR=$3

module load cesga/system miniconda3/22.11.1-1
conda activate "${CONDA_ENV_NAME}"

srun --export=ALL task.sh