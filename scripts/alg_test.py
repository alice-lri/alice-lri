import json
import argparse
import multiprocessing
import sqlite3
import subprocess
import sys

db_path = "../large/master.sqlite"
kitti_path = "../../../Datasets/LiDAR/kitti/"
durlar_path = "../../../Datasets/LiDAR/durlar/dataset/DurLAR/"

test_suite_query = """
WITH frame_diagnostics AS (
   SELECT df.relative_path, df.id, d.name AS dataset_name,
          frame_empirical.scanlines_count, d.laser_count,
          scanline.points_count
   FROM dataset_frame df
            JOIN dataset_frame_empirical frame_empirical ON frame_empirical.dataset_frame_id = df.id
            JOIN dataset_frame_scanline_info_empirical scanline ON scanline.dataset_frame_id = df.id
            JOIN dataset d ON df.dataset_id = d.id
)

SELECT * FROM (
                 SELECT DISTINCT dataset_name, relative_path
                 FROM frame_diagnostics
                 WHERE scanlines_count != laser_count AND dataset_name = 'kitti'
                 ORDER BY id LIMIT 25
             )
UNION
SELECT * FROM (
                 SELECT DISTINCT dataset_name, relative_path
                 FROM frame_diagnostics
                 WHERE scanlines_count != laser_count AND dataset_name = 'durlar'
                 ORDER BY id LIMIT 25
             )
UNION
SELECT * FROM (
                 SELECT DISTINCT dataset_name, relative_path
                 FROM frame_diagnostics
                 WHERE points_count < 10 AND dataset_name = 'kitti'
                 ORDER BY id LIMIT 25
             )
UNION
SELECT * FROM (
                 SELECT DISTINCT dataset_name, relative_path
                 FROM frame_diagnostics
                 WHERE points_count < 10 AND dataset_name = 'durlar'
                 ORDER BY id LIMIT 25
             )
UNION
SELECT * FROM (
                 SELECT DISTINCT dataset_name, relative_path
                 FROM frame_diagnostics
                 WHERE dataset_name = 'kitti'
                 ORDER BY id LIMIT 25
             )
UNION
SELECT * FROM (
                 SELECT DISTINCT dataset_name, relative_path
                 FROM frame_diagnostics
                 WHERE dataset_name = 'durlar'
                 ORDER BY id LIMIT 25
             );
"""

frame_gt_query = """
 SELECT gt.scanlines_count, gt.points_count
 FROM dataset d
          JOIN dataset_frame df ON df.dataset_id = d.id
          JOIN dataset_frame_empirical gt ON df.id = gt.dataset_frame_id
 WHERE d.name = ?
   AND df.relative_path = ?;
"""

scanlines_gt_query = """
 SELECT scanline_gt.scanline_idx,
        scanline_gt.points_count,
        scanline_gt.vertical_angle,
        scanline_gt.vertical_offset,
        scanline_gt.horizontal_resolution,
        scanline_gt.horizontal_offset,
        scanline_gt.horizontal_angle_offset
 FROM dataset d
          JOIN dataset_frame df ON df.dataset_id = d.id
          JOIN dataset_frame_scanline_info_empirical scanline_gt ON df.id = scanline_gt.dataset_frame_id
 WHERE d.name = ?
   AND df.relative_path = ?
 ORDER BY scanline_gt.laser_idx;
"""

def check_frame(indexed_item):
    idx, (path, query_result) = indexed_item
    out_path = f"/tmp/scanlines_alg_out_{idx}.json"

    print(f"Starting: {path}")
    subprocess.run(
        ["../cmake-build-release/examples/examples", path, "-1", out_path],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True
    )

    with open(out_path, "r") as f:
        data = json.load(f)

    with sqlite3.connect(db_path) as conn:
        cur = conn.cursor()
        cur.execute(frame_gt_query, (query_result[0], query_result[1]))
        scanlines_count, points_count = cur.fetchall()[0]

        cur.execute(scanlines_gt_query, (query_result[0], query_result[1]))
        scanlines_gt = cur.fetchall()

        cur.close()

    try:
        assert points_count == data["vertical"]["points_count"], \
            f"Points count mismatch: expected {points_count}, got {data['vertical']['points_count']}"
        assert scanlines_count == data["vertical"]["scanlines_count"], \
            f"Scanlines count mismatch: expected {scanlines_count}, got {data['vertical']['scanlines_count']}"

        scanlines_vert = data["vertical"]["scanlines_attributes"]
        scanlines_hor = data["horizontal"]["scanlines_attributes"]

        for i, (gt, vert, hor) in enumerate(zip(scanlines_gt, scanlines_vert, scanlines_hor)):
            assert vert["id"] == gt[0], f"[{i}] Scanline ID mismatch: expected {gt[0]}, got {vert['id']}"
            assert vert["count"] == gt[1], f"[{i}] Scanline point count mismatch: expected {gt[1]}, got {vert['count']}"
            assert abs(vert["angle"] - gt[2]) < 1e-3, f"[{i}] Angle mismatch: expected {gt[2]}, got {vert['angle']}"
            assert abs(vert["offset"] - gt[3]) < 1e-2, f"[{i}] Offset mismatch: expected {gt[3]}, got {vert['offset']}"
            assert hor["resolution"] == gt[4], f"[{i}] Resolution mismatch: expected {gt[4]}, got {hor['resolution']}"
            assert abs(hor["offset"] - gt[5]) < 1e-2, f"[{i}] Horizontal offset mismatch: expected {gt[5]}, got {hor['offset']}"
            assert abs(hor["thetaOffset"] - gt[6]) < 1e-3, f"[{i}] Theta offset mismatch: expected {gt[6]}, got {hor['thetaOffset']}"
    except AssertionError as e:
        print(f"ERROR: {path}: {e}")
        sys.exit()


def get_all_test_items():
    with sqlite3.connect(db_path) as conn:
        cur = conn.cursor()
        cur.execute(test_suite_query)
        test_suite_result = cur.fetchall()
        test_suite_paths = [kitti_path + row[1] if row[0] == "kitti" else durlar_path + row[1] for row in
                            test_suite_result]
        cur.close()

        print(f"Fetched {len(test_suite_paths)} items.")
    indexed_items = list(enumerate(zip(test_suite_paths, test_suite_result)))
    return indexed_items


def get_specific_test_item(args):
    dataset_name = args.dataset_name
    relative_path = args.relative_path
    with sqlite3.connect(db_path) as conn:
        cur = conn.cursor()
        cur.execute(frame_gt_query, (dataset_name, relative_path))
        if not cur.fetchone():
            print(f"No entry found for dataset '{dataset_name}' and path '{relative_path}'")
            sys.exit(1)
        cur.close()
    if dataset_name == "kitti":
        full_path = kitti_path + relative_path
    else:
        full_path = durlar_path + relative_path
    indexed_items = [(0, (full_path, (dataset_name, relative_path)))]
    return indexed_items


def main():
    parser = argparse.ArgumentParser(description="Run scanline algorithm test.")
    parser.add_argument("dataset_name", nargs="?", help="Dataset name (kitti or durlar)")
    parser.add_argument("relative_path", nargs="?", help="Relative path to a specific frame")
    args = parser.parse_args()

    if args.dataset_name and args.relative_path:
        indexed_items = get_specific_test_item(args)
    else:
        indexed_items = get_all_test_items()

    with multiprocessing.Pool(16) as pool:
        pool.map(check_frame, indexed_items)

if __name__ == "__main__":
    main()
