#!/bin/bash
set -eo pipefail
cd "$(dirname "$0")" || exit

source ../helper/paths.sh
source ../helper/multi_batch_job_header.sh
SRC_PATH="../../.."
EXECUTABLE_NAME="examples_sql"

echo "Building project..."
cmake -DCMAKE_BUILD_TYPE=Release -DLOG_LEVEL=INFO -DENABLE_PROFILING=ON -S "${SRC_PATH}" -B "${SRC_PATH}/build"
make -C "${SRC_PATH}/build"

source ../conda/init_conda.sh

echo "Preparing job..."
cp "${BASE_DB_DIR}/master.sqlite" "${ACTUAL_DB_DIR}/initial.sqlite"
python pre_job.py "${ACTUAL_DB_DIR}/initial.sqlite"

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