CREATE TABLE ri_experiment
(
    id integer PRIMARY KEY AUTOINCREMENT,
    timestamp text NOT NULL,
    label text NOT NULL,
    description text NOT NULL,
    commit_hash text NULL
);

CREATE TABLE ri_frame_result
(
    id integer PRIMARY KEY AUTOINCREMENT NOT NULL,
    experiment_id integer NOT NULL REFERENCES ri_experiment (id),
    dataset_frame_id integer NOT NULL REFERENCES dataset_frame (id),
    method text NOT NULL,
    ri_width integer NOT NULL,
    ri_height integer NOT NULL,
    original_points_count real NOT NULL,
    reconstructed_points_count real NOT NULL,
    reconstructed_to_original_mse real NOT NULL,
    original_to_reconstructed_mse real NOT NULL
);
CREATE INDEX ri_frame_result_experiment_id_idx ON ri_frame_result (experiment_id);
CREATE INDEX ri_frame_result_experiment_id_dataset_frame_id_idx ON ri_frame_result (experiment_id, dataset_frame_id);
