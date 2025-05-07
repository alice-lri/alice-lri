import sqlite3
import subprocess
import json

db_path = "../large/master.sqlite"
datasets = ["durlar", "kitti"]

wrong_resolution_query = """
                         SELECT DISTINCT d.name, df.relative_path, empirical.horizontal_resolution
                         FROM intrinsics_result_scanline_info scanline
                                  JOIN intrinsics_frame_result ifr ON ifr.id = scanline.intrinsics_result_id
                                  JOIN dataset_frame df ON df.id = ifr.dataset_frame_id
                                  JOIN dataset d ON d.id = df.dataset_id
                                  JOIN dataset_frame_scanline_info_empirical empirical
                                       ON df.id = empirical.dataset_frame_id AND
                                          empirical.scanline_idx = scanline.scanline_idx
                         WHERE ifr.experiment_id = 9
                           AND empirical.points_count > 30
                           AND (scanline.horizontal_resolution % empirical.horizontal_resolution != 0 OR scanline.horizontal_heuristic)
                         ORDER BY empirical.points_count DESC;
                         """

with sqlite3.connect(db_path) as conn:
    cur = conn.cursor()
    cur.execute(wrong_resolution_query)

    print("Fetching wrong items:")
    wrong_items = [(row[0], row[1], row[2]) for row in cur.fetchall()]
    print(f"Fetched {len(wrong_items)} wrong items.")

    for dataset, relative_path, gt_resolution in wrong_items:
        if dataset == "durlar":
            pre_path = "../../../Datasets/LiDAR/durlar/dataset/DurLAR/"
        else:
            pre_path = "../../../Datasets/LiDAR/kitti/"

        path = pre_path + relative_path

        # execute the compiled binary with the path as arg
        print(f"Executing {path}")
        subprocess.run(
            ["../cmake-build-release/examples/examples", path, "-1", "/tmp/scanlines_alg_out.json"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True
        )

        print("Verifying results:")
        with open("/tmp/scanlines_alg_out.json", "r") as f:
            data = json.load(f)
            for i, horizontal_scanline in enumerate(data["horizontal"]["scanlines_attributes"]):
                assert horizontal_scanline[
                           "resolution"] == gt_resolution, f"Wrong resolution for {path}, scanline {i}"

        print(f"OK for {path}!")

    cur.close()
    conn.commit()
