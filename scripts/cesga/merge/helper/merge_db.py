import sqlite3
import argparse
import os
import re
import shutil
import subprocess


def backup_db(merged_db_path):
    base_backup_db_path = merged_db_path + '.bak'
    backup_db_path = base_backup_db_path

    if os.path.exists(base_backup_db_path):
        index = 1
        while os.path.exists(f'{base_backup_db_path}.{index}'):
            index += 1
        backup_db_path = f'{base_backup_db_path}.{index}'

    if os.path.exists(merged_db_path):
        shutil.copy(merged_db_path, backup_db_path)


def get_db_files(folder_path):
    return [folder_path + "/" + f for f in os.listdir(folder_path) if re.fullmatch(r'\d+\.sqlite', f)]


def insert_merged_experiment(cursor, label, description):
    try:
        commit_hash = subprocess.check_output(['git', 'rev-parse', 'HEAD']).decode('utf-8').strip()
    except subprocess.CalledProcessError:
        commit_hash = None

    cursor.execute("""
        INSERT INTO experiment(timestamp, label, description, commit_hash) 
        VALUES (DATETIME('now', 'localtime', 'subsec'), ?, ?, ?)
    """, (label, description, commit_hash))

    return cursor.lastrowid


def assert_single_experiment(cursor):
    cursor.execute("SELECT experiment_id FROM intrinsics_frame_result")
    ids = set(row[0] for row in cursor.fetchall())

    if len(ids) != 1:
        raise ValueError(f"Unexpected experiment IDs found: {ids}")


def fetch_frames(cursor):
    cursor.execute("""
        SELECT id, dataset_frame_id, points_count, scanlines_count, vertical_iterations,
               unassigned_points, end_reason
        FROM intrinsics_frame_result
    """)
    return cursor.fetchall()


def fetch_scanlines(cursor, intrinsics_result_id):
    cursor.execute("""
        SELECT scanline_idx, points_count, vertical_offset, vertical_angle,
               vertical_ci_offset_lower, vertical_ci_offset_upper,
               vertical_ci_angle_lower, vertical_ci_angle_upper,
               vertical_theoretical_angle_bottom_lower, vertical_theoretical_angle_bottom_upper,
               vertical_theoretical_angle_top_lower, vertical_theoretical_angle_top_upper,
               vertical_uncertainty, vertical_last_scanline, vertical_hough_votes,
               vertical_hough_hash, horizontal_offset, horizontal_resolution, horizontal_heuristic
        FROM intrinsics_result_scanline_info
        WHERE intrinsics_result_id = ?
    """, (intrinsics_result_id,))
    return cursor.fetchall()


def insert_frame(cursor, merged_experiment_id, frame_data):
    cursor.execute("""
        INSERT INTO intrinsics_frame_result(
            experiment_id, dataset_frame_id, points_count, scanlines_count,
            vertical_iterations, unassigned_points, end_reason
        ) VALUES (?, ?, ?, ?, ?, ?, ?)
    """, (merged_experiment_id, *frame_data))
    return cursor.lastrowid


def insert_scanlines(cursor, frame_id, scanlines):
    data = [(frame_id, *scanline) for scanline in scanlines]
    cursor.executemany("""
        INSERT INTO intrinsics_result_scanline_info(
            intrinsics_result_id, scanline_idx, points_count, vertical_offset, vertical_angle,
            vertical_ci_offset_lower, vertical_ci_offset_upper, vertical_ci_angle_lower,
            vertical_ci_angle_upper, vertical_theoretical_angle_bottom_lower,
            vertical_theoretical_angle_bottom_upper, vertical_theoretical_angle_top_lower,
            vertical_theoretical_angle_top_upper, vertical_uncertainty, vertical_last_scanline,
            vertical_hough_votes, vertical_hough_hash, horizontal_offset, horizontal_resolution,
            horizontal_heuristic
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    """, data)


def merge_experiment_databases(db_files, master_db_path, label, description):
    master_conn = sqlite3.connect(master_db_path)
    master_c = master_conn.cursor()
    merged_experiment_id = insert_merged_experiment(master_c, label, description)

    files_count = len(db_files)

    for file_index, db_file in enumerate(db_files):
        print(f"Merging experiments database {file_index + 1}/{files_count}")

        with sqlite3.connect(db_file) as conn:
            c = conn.cursor()
            assert_single_experiment(c)
            frames = fetch_frames(c)

            for frame_id_original, *frame_data in frames:
                frame_id_new = insert_frame(master_c, merged_experiment_id, frame_data)
                scanlines = fetch_scanlines(c, frame_id_original)
                insert_scanlines(master_c, frame_id_new, scanlines)

    master_conn.commit()
    master_conn.close()


def fetch_gt_frames(cursor):
    cursor.execute("""
        SELECT dataset_frame_id, points_count, scanlines_count FROM dataset_frame_empirical
    """)

    return cursor.fetchall()


def fetch_gt_scanlines(cursor):
    cursor.execute("""
        SELECT dataset_frame_id,
               scanline_idx,
               laser_idx,
               points_count,
               vertical_offset,
               vertical_angle,
               horizontal_offset,
               horizontal_resolution
        FROM dataset_frame_scanline_info_empirical
    """)

    return cursor.fetchall()


def insert_gt_frames(cursor, data):
    cursor.executemany("""
        INSERT INTO dataset_frame_empirical(dataset_frame_id, points_count, scanlines_count)
        VALUES (?, ?, ?)
    """, data)


def insert_gt_scanlines(cursor, data):
    cursor.executemany("""
        INSERT INTO dataset_frame_scanline_info_empirical(dataset_frame_id, scanline_idx, laser_idx, points_count,
                                                          vertical_offset, vertical_angle, horizontal_offset,
                                                          horizontal_resolution)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?)
    """, data)


def merge_ground_truth_databases(db_files, master_db_path):
    master_conn = sqlite3.connect(master_db_path)
    master_c = master_conn.cursor()

    files_count = len(db_files)

    for file_index, db_file in enumerate(db_files):
        print(f"Merging ground truth database {file_index + 1}/{files_count}")

        with sqlite3.connect(db_file) as conn:
            c = conn.cursor()
            frames = fetch_gt_frames(c)
            scanlines = fetch_gt_scanlines(c)

            insert_gt_frames(master_c, frames)
            insert_gt_scanlines(master_c, scanlines)

    master_conn.commit()
    master_conn.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Merge SQLite databases.")
    parser.add_argument("part_dbs_folder_path")
    parser.add_argument("master_db_path")
    parser.add_argument("--type", choices=["experiments", "ground_truth"], required=True,
                        help="Type of databases to merge: experiments or ground_truth")
    parser.add_argument("--label")
    parser.add_argument("--description")
    args = parser.parse_args()

    if args.type == "experiments":
        if not args.label or not args.description:
            parser.error("--label and --description required when --type is 'experiments'")

    print("Backing up database...")
    backup_db(args.master_db_path)
    db_files = get_db_files(args.part_dbs_folder_path)

    if args.type == "experiments":
        merge_experiment_databases(db_files, args.master_db_path, args.label, args.description)
    elif args.type == "ground_truth":
        merge_ground_truth_databases(db_files, args.master_db_path)
