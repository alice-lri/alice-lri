CREATE TABLE compression_experiment
(
    id integer PRIMARY KEY AUTOINCREMENT,
    timestamp text NOT NULL,
    label text NOT NULL,
    description text NOT NULL,
    commit_hash text NULL
);

CREATE TABLE compression_frame_result
(
    id integer PRIMARY KEY AUTOINCREMENT NOT NULL,
    experiment_id integer NOT NULL REFERENCES compression_experiment (id),
    dataset_frame_id integer NOT NULL REFERENCES dataset_frame (id),
    horizontal_step real NOT NULL,
    vertical_step real NOT NULL,
    tile_size integer NOT NULL,
    error_threshold real NOT NULL,
    original_points_count integer NOT NULL,
    naive_points_count integer NOT NULL,
    original_size_bytes integer NOT NULL,
    naive_size_bytes integer NOT NULL,
    accurate_size_bytes integer NOT NULL,
    accurate_points_count integer NOT NULL,
    naive_to_original_mse real NOT NULL,
    original_to_naive_mse real NOT NULL,
    accurate_to_original_mse real NOT NULL,
    original_to_accurate_mse real NOT NULL
);
CREATE INDEX compression_frame_result_experiment_id_idx ON compression_frame_result (experiment_id);
CREATE INDEX compression_frame_result_experiment_id_dataset_frame_id_idx ON compression_frame_result (experiment_id, dataset_frame_id);
