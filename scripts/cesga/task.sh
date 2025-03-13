#!/bin/bash

CONDA_ENV_NAME=$1
EXECUTABLE_PATH=$2
DB_DIR=$3

if [ -z "$4" ] || [ -z "$5" ]; then
  TASK_INDEX=$SLURM_PROCID
  TASK_COUNT=$SLURM_NTASKS
else
  ARRAY_INDEX=$4
  ARRAY_COUNT=$5
  TASK_INDEX=$(( ARRAY_INDEX * SLURM_NTASKS + SLURM_PROCID ))
  TASK_COUNT=$(( ARRAY_COUNT * SLURM_NTASKS ))
fi

EXEC_DIR=$(dirname "$EXECUTABLE_PATH")
EXEC_FILE=$(basename "$EXECUTABLE_PATH")

cd "$EXEC_DIR" || { echo "Failed to cd into $EXEC_DIR"; exit 1; }
cp "${DB_DIR}/initial.sqlite" "${DB_DIR}/${TASK_INDEX}.sqlite"

eval "$(conda shell.bash hook)"
conda activate "${CONDA_ENV_NAME}"

echo "Running task $TASK_INDEX of $TASK_COUNT..."
./"$EXEC_FILE" "$TASK_INDEX" "$TASK_COUNT"
