#!/bin/bash
set -eo pipefail

## Adjust these as needed
CONDA_ENV_NAME="accurate_ri_env"
BASE_DB_DIR="${STORE2}/accurate_ri_db"
KITTI_PATH="${STORE2}/datasets_lidar/kitti"
DURLAR_PATH="${STORE2}/datasets_lidar/durlar/dataset/DurLAR"
SRC_PATH="../.."
EXECUTABLE_NAME="examples_sql"
JOB_COUNT=32

cd "$(dirname "$0")" || exit

if [ -n "$1" ]; then
  BATCH_ID=$1
  RESUME_BATCH=true
else
  BATCH_ID="$(date +'%Y%m%d_%H%M%S_%3N')"
  RESUME_BATCH=false
fi

LOGS_DIR="logs/${BATCH_ID}"
ACTUAL_DB_DIR="${BASE_DB_DIR}/${BATCH_ID}"

if [ "$RESUME_BATCH" = true ]; then
  if [ ! -d "${LOGS_DIR}" ]; then
    echo "Log directory ${LOGS_DIR} does not exist."
    exit 1
  fi

  if [ ! -d "${ACTUAL_DB_DIR}" ]; then
    echo "DB directory ${ACTUAL_DB_DIR} does not exist."
    exit 1
  fi

  JOBS_TO_RUN=()
  for i in $(seq 0 $((JOB_COUNT - 1))); do
    if [ ! -f "${ACTUAL_DB_DIR}/job_${i}.success" ]; then
      JOBS_TO_RUN+=("$i")
    fi
  done
else
    for i in $(seq 0 $((JOB_COUNT - 1))); do
      JOBS_TO_RUN+=("$i")
    done
fi

echo "Will run jobs: " "${JOBS_TO_RUN[@]}"
echo "Continue? (y/n)"
read -r CONTINUE

if [ "$CONTINUE" != "y" ]; then
  echo "Aborting."
  exit 1
fi

mkdir -p "${ACTUAL_DB_DIR}"
mkdir -p .cache

echo "Building project..."
cmake -DCMAKE_BUILD_TYPE=Release -DLOG_LEVEL=INFO -DENABLE_PROFILING=ON -S "${SRC_PATH}" -B "${SRC_PATH}/build"
make -C "${SRC_PATH}/build"

module load cesga/system miniconda3/22.11.1-1

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

mkdir -p logs

for i in "${JOBS_TO_RUN[@]}"; do
  echo "Launching job ${i}..."
  sbatch --job-name="accurate_ri_${i}" \
   job.sh "${CONDA_ENV_NAME}" "${SRC_PATH}/build/examples/${EXECUTABLE_NAME}" "${ACTUAL_DB_DIR}" \
    "${i}" "${JOB_COUNT}"
done