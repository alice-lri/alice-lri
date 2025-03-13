#!/bin/bash

EXEC_DIR=$(dirname "$EXECUTABLE_PATH")
EXEC_FILE=$(basename "$EXECUTABLE_PATH")

cd "$EXEC_DIR" || { echo "Failed to cd into $EXEC_DIR"; exit 1; }
cp "${DB_DIR}/initial.sqlite" "${DB_DIR}/${SLURM_PROCID}.sqlite"

eval "$(conda shell.bash hook)"
conda activate "${CONDA_ENV_NAME}"

./"$EXEC_FILE" "$SLURM_PROCID" "$SLURM_NTASKS"
