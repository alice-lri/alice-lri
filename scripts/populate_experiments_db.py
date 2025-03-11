import glob
import sqlite3

# list contents that match
prefixes = [
    "/home/samuel.soutullo/Datasets/LiDAR/durlar/dataset/DurLAR/",
    "/home/samuel.soutullo/Datasets/LiDAR/kitti/"
]

patterns = [
    "*/ouster_points/data/*.bin",
    "*/*/velodyne_points/data/*.bin"
]

datasets = ["durlar", "kitti"]
scanlines_counts = [128, 64]

for prefix, pattern, dataset, scanlines_count in zip(prefixes, patterns, datasets, scanlines_counts):
    files = glob.glob(prefix + pattern)
    files_relative = [f.replace(prefix, "") for f in files]

    conn = sqlite3.connect("../examples/experiments/experiments.sqlite")
    cur = conn.cursor()

    cur.execute('''
    INSERT OR IGNORE INTO dataset(name, typical_scanlines_count)
    VALUES (?, ?) 
    ''', (dataset, scanlines_count))

    cur.execute("SELECT id FROM dataset WHERE name = ?", (dataset,))
    dataset_id = cur.fetchone()[0]

    frames_insert_data = [(dataset_id, f) for f in files_relative]

    cur.executemany('''
    INSERT OR IGNORE INTO dataset_frame(dataset_id, relative_path)
    VALUES (?, ?) 
    ''', frames_insert_data)

    conn.commit()
    conn.close()
