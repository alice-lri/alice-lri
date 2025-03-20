#!/bin/bash
set -eo pipefail
cd "$(dirname "$0")" || exit

source helper/merge_header.sh

echo "Merging experiments databases from ${TARGET_DIR}..."

source init_conda.sh
python helper/merge_db.py "$TARGET_DIR" "$BASE_DB_DIR/master.sqlite" --type="experiments"

echo "Experiments database merged successfully."
