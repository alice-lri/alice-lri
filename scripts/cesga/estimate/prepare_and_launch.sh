#!/bin/bash
set -eo pipefail
cd "$(dirname "$0")" || exit

source ../helper/paths.sh
SRC_PATH="../../.."
EXECUTABLE_NAME="examples_sql"
JOB_COUNT=32

if [ -n "$1" ]; then
  BATCH_ID=$1
  RESUME_BATCH=true
else
  BATCH_ID="$(date +'%Y%m%d_%H%M%S_%3N')"
  RESUME_BATCH=false
fi

ACTUAL_LOGS_DIR="${BASE_LOGS_DIR}/${BATCH_ID}"
ACTUAL_DB_DIR="${BASE_DB_DIR}/${BATCH_ID}"

if [ "$RESUME_BATCH" = true ]; then
  if [ ! -d "${ACTUAL_LOGS_DIR}" ]; then
    echo "Log directory ${ACTUAL_LOGS_DIR} does not exist."
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
mkdir -p "${ACTUAL_LOGS_DIR}"

echo "Building project..."
cmake -DCMAKE_BUILD_TYPE=Release -DLOG_LEVEL=INFO -DENABLE_PROFILING=ON -S "${SRC_PATH}" -B "${SRC_PATH}/build"
make -C "${SRC_PATH}/build"

source ../conda/init_conda.sh

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


for i in "${JOBS_TO_RUN[@]}"; do
  echo "Launching job ${i}..."
  sbatch --job-name="accurate_ri_${i}" -o "${ACTUAL_LOGS_DIR}/${i}.log" -e "${ACTUAL_LOGS_DIR}/${i}.log"\
   job.sh "${CONDA_ENV_NAME}" "${SRC_PATH}/build/examples/${EXECUTABLE_NAME}" "${ACTUAL_DB_DIR}" \
    "${i}" "${JOB_COUNT}"
done