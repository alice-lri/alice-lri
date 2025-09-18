import glob
import sqlite3
from populate_ground_truth_db import get_sensor_properties

db_path = "../large/master.sqlite"
datasets = ["durlar", "kitti"]

with sqlite3.connect(db_path) as conn:
    cur = conn.cursor()

    for dataset in datasets:
        v_offsets, v_angles, h_offsets, h_resolutions = get_sensor_properties(dataset)

        cur.execute("SELECT id FROM dataset WHERE name = ?", (dataset,))
        dataset_id = cur.fetchone()[0]

        for laser_idx, (v_offset, v_angle, h_offset, h_resolution) in enumerate(zip(v_offsets, v_angles, h_offsets, h_resolutions)):
            cur.execute("""
                INSERT OR IGNORE INTO dataset_scanline_info(dataset_id, laser_idx, vertical_angle, vertical_offset, 
                    horizontal_resolution, horizontal_offset)
                VALUES (?, ?, ?, ?, ?, ?)
            """, (dataset_id, laser_idx, v_angle, v_offset, h_resolution, h_offset))

    cur.close()
    conn.commit()
