#!/bin/bash
set -eo pipefail
source ../helper/paths.sh

CONDA_ENV_NAME=$1
DB_DIR=$2
TASK_INDEX=$3
TASK_COUNT=$4

DB_FILE_PATH="${DB_DIR}/${TASK_INDEX}.sqlite"
SUCCESS_FILE_PATH="${DB_DIR}/task_${TASK_INDEX}.success"
TRACE_FOLDER_PATH="${DB_DIR}/traces"
TRACE_FILE_PATH="${TRACE_FOLDER_PATH}/${TASK_INDEX}.log"

rm -f "${DB_FILE_PATH}"
rm -f "${SUCCESS_FILE_PATH}"
rm -f "${TRACE_FILE_PATH}"

cp "${DB_DIR}/initial.sqlite" "${DB_FILE_PATH}"

mkdir -p "${TRACE_FOLDER_PATH}"

eval "$(conda shell.bash hook)"
conda activate "${CONDA_ENV_NAME}"

echo "Running task $TASK_INDEX of $TASK_COUNT..."
python populate_ground_truth_db.py "$TASK_INDEX" "$TASK_COUNT" \
  --db_path="${DB_FILE_PATH}" \
  --kitti_root="${KITTI_PATH}" \
  --durlar_root="${DURLAR_PATH}" 2>&1 | tee "${TRACE_FILE_PATH}"

touch "${SUCCESS_FILE_PATH}"