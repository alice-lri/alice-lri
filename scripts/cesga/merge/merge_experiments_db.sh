#!/bin/bash
set -eo pipefail
cd "$(dirname "$0")" || exit

source helper/merge_header.sh

read -rp "Experiment label: " LABEL
read -rp "Experiment description: " DESCRIPTION

echo "Merging experiments databases from ${TARGET_DIR}..."

source ../conda/init_conda.sh
python helper/merge_db.py "$TARGET_DIR" "$BASE_DB_DIR/master.sqlite" --type="experiments" \
  --label="$LABEL" --description="$DESCRIPTION"

echo "Experiments database merged successfully."
