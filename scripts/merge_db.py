import glob
import sqlite3


def get_db_files(path_pattern, exclude_substring):
    return [f for f in glob.glob(path_pattern) if exclude_substring not in f]


def insert_merged_experiment(cursor):
    cursor.execute("INSERT INTO experiment(timestamp) VALUES (DATETIME('now'))")
    return cursor.lastrowid


def assert_single_experiment(cursor):
    cursor.execute("SELECT experiment_id FROM intrinsics_frame_result")
    ids = set(row[0] for row in cursor.fetchall())

    if len(ids) != 1:
        raise ValueError(f"Multiple experiment IDs found: {ids}")


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


def merge_databases(db_files, merged_db_path='../large/merge.sqlite'):
    merge_conn = sqlite3.connect(merged_db_path)
    merge_c = merge_conn.cursor()
    merged_experiment_id = insert_merged_experiment(merge_c)

    for db_file in db_files:
        with sqlite3.connect(db_file) as conn:
            c = conn.cursor()
            assert_single_experiment(c)
            frames = fetch_frames(c)

            for frame_id_original, *frame_data in frames:
                frame_id_new = insert_frame(merge_c, merged_experiment_id, frame_data)
                scanlines = fetch_scanlines(c, frame_id_original)
                insert_scanlines(merge_c, frame_id_new, scanlines)

    merge_conn.commit()
    merge_conn.close()


if __name__ == "__main__":
    db_files = get_db_files('../large/*.sqlite', exclude_substring='initial')
    merge_databases(db_files)
