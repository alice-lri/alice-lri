CREATE TABLE dataset
(
    id integer PRIMARY KEY AUTOINCREMENT,
    name text NOT NULL,
    typical_scanlines_count integer NOT NULL
);

CREATE TABLE dataset_frame
(
    id integer PRIMARY KEY AUTOINCREMENT,
    dataset_id integer NOT NULL REFERENCES dataset (id),
    relative_path text NOT NULL
);

CREATE TABLE dataset_frame_empirical
(
    id integer PRIMARY KEY AUTOINCREMENT,
    dataset_frame_id integer NOT NULL REFERENCES dataset_frame (id),
    points_count integer NOT NULL,
    scanlines_count integer NOT NULL
);

CREATE TABLE dataset_scanline_info
(
    id integer PRIMARY KEY AUTOINCREMENT,
    dataset_id integer NOT NULL REFERENCES dataset (id),
    scanline_idx integer NOT NULL,
    vertical_angle real NOT NULL,
    vertical_offset real NOT NULL,
    horizontal_resolution integer NOT NULL,
    horizontal_offset real NOT NULL
);

CREATE TABLE dataset_frame_scanline_info_empirical
(
    id integer PRIMARY KEY AUTOINCREMENT,
    dataset_frame_id integer NOT NULL REFERENCES dataset_frame (id),
    scanline_idx integer NOT NULL,
    points_count integer NOT NULL,
    vertical_offset real NOT NULL,
    vertical_angle real NOT NULL,
    horizontal_offset real NOT NULL,
    horizontal_resolution integer NOT NULL
);

CREATE TABLE intrinsics_frame_result
(
    id integer PRIMARY KEY AUTOINCREMENT NOT NULL,
    dataset_frame_id integer NOT NULL REFERENCES dataset_frame (id),
    points_count integer NOT NULL,
    scanlines_count integer NOT NULL,
    vertical_iterations integer NOT NULL,
    unassigned_points integer NOT NULL,
    end_reason text CHECK ( end_reason IN ('ALL_ASSIGNED', 'MAX_ITERATIONS', 'NO_MORE_PEAKS') ) NOT NULL
);

CREATE TABLE intrinsics_result_scanline_info
(
    id integer PRIMARY KEY AUTOINCREMENT,
    intrinsics_result_id integer NOT NULL REFERENCES intrinsics_frame_result (id),
    scanline_idx integer NOT NULL,
    points_count integer NOT NULL,
    vertical_offset real NOT NULL,
    vertical_angle real NOT NULL,
    vertical_ci_offset_lower real NOT NULL,
    vertical_ci_offset_upper real NOT NULL,
    vertical_ci_angle_lower real NOT NULL,
    vertical_ci_angle_upper real NOT NULL,
    vertical_theoretical_angle_bottom_lower real NOT NULL,
    vertical_theoretical_angle_bottom_upper real NOT NULL,
    vertical_theoretical_angle_top_lower real NOT NULL,
    vertical_theoretical_angle_top_upper real NOT NULL,
    vertical_uncertainty real NOT NULL,
    vertical_last_scanline boolean NOT NULL,
    vertical_hough_votes real NOT NULL,
    vertical_hough_hash integer NOT NULL,
    horizontal_offset real NOT NULL,
    horizontal_resolution integer NOT NULL,
    horizontal_heuristic boolean NOT NULL
);
