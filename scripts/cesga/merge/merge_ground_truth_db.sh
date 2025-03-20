#!/bin/bash
set -eo pipefail
cd "$(dirname "$0")" || exit

source helper/merge_header.sh

echo "Merging ground truth databases from ${TARGET_DIR}..."

source ../conda/init_conda.sh
python helper/merge_db.py "$TARGET_DIR" "$BASE_DB_DIR/master.sqlite" --type="ground_truth"

echo "Ground truth database merged successfully."
