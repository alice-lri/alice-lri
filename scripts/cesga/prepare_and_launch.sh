#!/bin/bash
set -e

## Adjust these as needed
CONDA_ENV_NAME="accurate_ri_env"
BASE_DB_DIR="${STORE2}/accurate_ri"
KITTI_PATH="${STORE2}/datasets_lidar/kitti"
DURLAR_PATH="${STORE2}/datasets_lidar/durlar/dataset/DurLAR"
SRC_PATH="../.."
EXECUTABLE_NAME="examples_sql"

cd "$(dirname "$0")" || exit
module load cesga/system miniconda3/22.11.1-1

ACTUAL_DB_DIR="${BASE_DB_DIR}/$(date +'%Y%m%d_%H%M%S_%3N')"
mkdir -p "${ACTUAL_DB_DIR}"
mkdir -p .cache

if ! command -v conda &> /dev/null
then
    echo "Conda could not be found"
    exit
fi

if ! conda env list | awk '{print $1}' | grep -wq "${CONDA_ENV_NAME}"; then
    echo "Conda environment \`${CONDA_ENV_NAME}\` does not exist. Creating..."
    conda create --name "${CONDA_ENV_NAME}" -y
    rm -f .cache/conda_env
fi

if [ conda_env.yml -nt .cache/conda_env ]; then
  echo "Updating conda environment..."
  conda env update --file conda_env.yml --name "${CONDA_ENV_NAME}" --prune
  touch .cache/conda_env
else
  echo "Conda environment is up to date."
fi

conda activate "${CONDA_ENV_NAME}"

echo "Building project..."
cmake -DCMAKE_BUILD_TYPE=Release -DLOG_LEVEL=INFO -DENABLE_PROFILING=ON -S "${SRC_PATH}" -B "${SRC_PATH}/build"
make -C "${SRC_PATH}/build"

echo "Preparing job..."
python pre_job.py

cp ../../examples/experiments/experiments.sqlite "${ACTUAL_DB_DIR}/initial.sqlite"

jq -n \
  --arg db_dir "$ACTUAL_DB_DIR" \
  --arg durlar "$DURLAR_PATH" \
  --arg kitti "$KITTI_PATH" \
  '{
    db_dir: $db_dir,
    dataset_root_path: {
      durlar: $durlar,
      kitti: $kitti
    }
  }' > "${SRC_PATH}/build/examples/config.json"


echo "Launching job..."
sbatch job.sh "${CONDA_ENV_NAME}" "${SRC_PATH}/build/examples/${EXECUTABLE_NAME}" "${ACTUAL_DB_DIR}"