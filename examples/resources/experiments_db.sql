CREATE TABLE dataset
(
    id integer PRIMARY KEY AUTOINCREMENT,
    name text NOT NULL UNIQUE,
    laser_count integer NOT NULL
);
CREATE INDEX dataset_name_idx ON dataset (name);

CREATE TABLE dataset_frame
(
    id integer PRIMARY KEY AUTOINCREMENT,
    dataset_id integer NOT NULL REFERENCES dataset (id),
    relative_path text NOT NULL UNIQUE
);
CREATE INDEX dataset_frame_dataset_id_idx ON dataset_frame (dataset_id);
CREATE INDEX dataset_frame_relative_path_idx ON dataset_frame (relative_path);

CREATE TABLE dataset_scanline_info
(
    id integer PRIMARY KEY AUTOINCREMENT,
    dataset_id integer NOT NULL REFERENCES dataset (id),
    laser_idx integer NOT NULL,
    vertical_angle real NOT NULL,
    vertical_offset real NOT NULL,
    horizontal_resolution integer NOT NULL,
    horizontal_offset real NOT NULL,

    UNIQUE (dataset_id, laser_idx)
);
CREATE INDEX dataset_scanline_info_dataset_id ON dataset_scanline_info (dataset_id);
CREATE INDEX dataset_scanline_info_dataset_id_laser_idx ON dataset_scanline_info (dataset_id, laser_idx);

CREATE TABLE dataset_frame_empirical
(
    id integer PRIMARY KEY AUTOINCREMENT,
    dataset_frame_id integer NOT NULL UNIQUE REFERENCES dataset_frame (id),
    points_count integer NOT NULL,
    scanlines_count integer NOT NULL
);
CREATE INDEX dataset_frame_empirical_dataset_frame_id_idx ON dataset_frame_empirical (dataset_frame_id);

CREATE TABLE dataset_frame_scanline_info_empirical
(
    id integer PRIMARY KEY AUTOINCREMENT,
    dataset_frame_id integer NOT NULL REFERENCES dataset_frame (id),
    scanline_idx integer NOT NULL,
    laser_idx integer NOT NULL,
    points_count integer NOT NULL,
    vertical_offset real NOT NULL,
    vertical_angle real NOT NULL,
    horizontal_offset real NOT NULL,
    horizontal_resolution integer NOT NULL,

    UNIQUE (dataset_frame_id, scanline_idx),
    UNIQUE (dataset_frame_id, laser_idx)
);
CREATE INDEX dataset_frame_scanline_info_empirical_dataset_frame_id_idx ON dataset_frame_scanline_info_empirical (dataset_frame_id);
CREATE INDEX dataset_frame_scanline_info_empirical_dataset_frame_id_scanline_idx ON dataset_frame_scanline_info_empirical (dataset_frame_id, scanline_idx);

CREATE TABLE experiment
(
    id integer PRIMARY KEY AUTOINCREMENT,
    timestamp text NOT NULL
);

CREATE TABLE intrinsics_frame_result
(
    id integer PRIMARY KEY AUTOINCREMENT NOT NULL,
    experiment_id integer NOT NULL REFERENCES experiment (id),
    dataset_frame_id integer NOT NULL REFERENCES dataset_frame (id),
    points_count integer NOT NULL,
    scanlines_count integer NOT NULL,
    vertical_iterations integer NOT NULL,
    unassigned_points integer NOT NULL,
    end_reason text CHECK ( end_reason IN ('ALL_ASSIGNED', 'MAX_ITERATIONS', 'NO_MORE_PEAKS') ) NOT NULL,

    UNIQUE (experiment_id, dataset_frame_id)
);
CREATE INDEX intrinsics_frame_result_experiment_id_idx ON intrinsics_frame_result (experiment_id);
CREATE INDEX intrinsics_frame_result_experiment_id_dataset_frame_id_idx ON intrinsics_frame_result (experiment_id, dataset_frame_id);

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
    vertical_hough_hash text NOT NULL,
    horizontal_offset real NOT NULL,
    horizontal_resolution integer NOT NULL,
    horizontal_heuristic boolean NOT NULL,

    UNIQUE (intrinsics_result_id, scanline_idx)
);
CREATE INDEX intrinsics_result_scanline_info_intrinsics_result_id_idx ON intrinsics_result_scanline_info (intrinsics_result_id);
CREATE INDEX intrinsics_result_scanline_info_intrinsics_result_id_scanline_idx_idx ON intrinsics_result_scanline_info (intrinsics_result_id, scanline_idx);