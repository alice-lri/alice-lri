import sqlite3
import subprocess
import json
from concurrent.futures import ProcessPoolExecutor
import sys
import os

db_path = "../large/master.sqlite"

wrong_resolution_query = """
                         SELECT DISTINCT d.name, df.relative_path, empirical.horizontal_resolution
                         FROM intrinsics_result_scanline_info scanline
                                  JOIN intrinsics_frame_result ifr ON ifr.id = scanline.intrinsics_result_id
                                  JOIN dataset_frame df ON df.id = ifr.dataset_frame_id
                                  JOIN dataset d ON d.id = df.dataset_id
                                  JOIN dataset_frame_scanline_info_empirical empirical
                                       ON df.id = empirical.dataset_frame_id AND
                                          empirical.scanline_idx = scanline.scanline_idx
                         WHERE ifr.experiment_id = 11
                           AND empirical.points_count > 30
                           AND (scanline.horizontal_resolution != empirical.horizontal_resolution OR scanline.horizontal_heuristic)
                         ORDER BY empirical.points_count DESC; \
                         """

def verify_and_run(indexed_item):
    idx, (dataset, relative_path, gt_resolution) = indexed_item
    pre_path = "../../../Datasets/LiDAR/durlar/dataset/DurLAR/" if dataset == "durlar" else "../../../Datasets/LiDAR/kitti/"
    path = pre_path + relative_path

    out_path = f"/tmp/scanlines_alg_out_{os.getpid()}_{idx}.json"

    subprocess.run(
        ["../cmake-build-release/examples/examples", path, "-1", out_path],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True
    )

    try:
        with open(out_path, "r") as f:
            data = json.load(f)
            for i, horizontal_scanline in enumerate(data["horizontal"]["scanlines_attributes"]):
                if horizontal_scanline["resolution"] != gt_resolution:
                    raise ValueError(f"Wrong resolution for {path}, scanline {i}")
    except Exception as e:
        print(e)

    os.remove(out_path)
    print(f"OK for {path}!")

if __name__ == "__main__":
    with sqlite3.connect(db_path) as conn:
        cur = conn.cursor()
        cur.execute(wrong_resolution_query)
        wrong_items = [(row[0], row[1], row[2]) for row in cur.fetchall()]
        cur.close()
        conn.commit()

    print(f"Fetched {len(wrong_items)} wrong items.")

    try:
        indexed_items = list(enumerate(wrong_items))

        with ProcessPoolExecutor(max_workers=16) as executor:
            for result in executor.map(verify_and_run, indexed_items):
                pass

    except Exception as e:
        print(f"Aborting due to error: {e}", file=sys.stderr)
        sys.exit(1)
