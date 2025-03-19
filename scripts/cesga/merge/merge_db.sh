#!/bin/bash
set -eo pipefail
cd "$(dirname "$0")" || exit

source ../helper/paths.sh

if [ -z "$1" ]; then
  TARGET_DIR=$(ls -td1 "${BASE_DB_DIR}"/*/ | head -1)
  echo "No target directory specified. Using latest: ${TARGET_DIR}"
  echo "Continue? (y/n)"
  read -r CONTINUE
  if [ "$CONTINUE" != "y" ]; then
    echo "Aborting."
    exit 1
  fi
else
  TARGET_DIR="$BASE_DB_DIR/$1"
fi

echo "Merging databases from ${TARGET_DIR}..."

source init_conda.sh
python merge_db.py "$TARGET_DIR" "$BASE_DB_DIR/master.sqlite"

echo "Database merged successfully."
