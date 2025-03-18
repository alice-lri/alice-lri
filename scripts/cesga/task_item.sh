#!/bin/bash
set -eo pipefail

CONDA_ENV_NAME=$1
EXECUTABLE_PATH=$2
DB_DIR=$3
TASK_INDEX=$4
TASK_COUNT=$5

EXEC_DIR=$(dirname "$EXECUTABLE_PATH")
EXEC_FILE=$(basename "$EXECUTABLE_PATH")

cd "$EXEC_DIR" || { echo "Failed to cd into $EXEC_DIR"; exit 1; }

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
./"$EXEC_FILE" "$TASK_INDEX" "$TASK_COUNT" 2>&1 | tee "${TRACE_FILE_PATH}"

touch "${SUCCESS_FILE_PATH}"