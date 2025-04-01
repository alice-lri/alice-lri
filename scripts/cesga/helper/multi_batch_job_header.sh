#!/bin/bash
set -eo pipefail

JOB_COUNT=32

BATCH_ID="$(date +'%Y%m%d_%H%M%S_%3N')"
RESUME_BATCH=false
USER_REQUESTED_JOBS=()
JOBS_TO_RUN=()

if [ -n "$1" ]; then
  BATCH_ID=$1
  RESUME_BATCH=true

  shift
  for arg in "$@"; do
    if [[ "$arg" =~ ^[0-9]+$ ]]; then
      USER_REQUESTED_JOBS+=("$arg")
    else
      echo "Invalid job index: $arg"
      exit 1
    fi
  done
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

  if [ "${#USER_REQUESTED_JOBS[@]}" -gt 0 ]; then
    JOBS_TO_RUN=("${USER_REQUESTED_JOBS[@]}")
  else
    for i in $(seq 0 $((JOB_COUNT - 1))); do
      if [ ! -f "${ACTUAL_DB_DIR}/job_${i}.success" ]; then
        JOBS_TO_RUN+=("$i")
      fi
    done
  fi
else
  if [ "${#USER_REQUESTED_JOBS[@]}" -gt 0 ]; then
    JOBS_TO_RUN=("${USER_REQUESTED_JOBS[@]}")
  else
    for i in $(seq 0 $((JOB_COUNT - 1))); do
      JOBS_TO_RUN+=("$i")
    done
  fi
fi

echo "Will run jobs: ${JOBS_TO_RUN[*]}"
echo "Continue? (y/n)"
read -r CONTINUE

if [ "$CONTINUE" != "y" ]; then
  echo "Aborting."
  exit 1
fi

mkdir -p "${ACTUAL_DB_DIR}"
mkdir -p "${ACTUAL_LOGS_DIR}"
