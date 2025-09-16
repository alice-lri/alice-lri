CREATE TABLE dataset_laser_gt
(
    id integer PRIMARY KEY AUTOINCREMENT,
    dataset_id integer NOT NULL REFERENCES dataset (id),
    laser_idx integer NOT NULL,
    vertical_offset real NOT NULL,
    vertical_angle real NOT NULL,
    horizontal_offset real NOT NULL,
    horizontal_resolution integer NOT NULL,
    horizontal_angle_offset real NOT NULL,
    UNIQUE (dataset_id, laser_idx)
);
CREATE INDEX dataset_laser_gt_dataset_id_idx ON dataset_laser_gt (dataset_id);

ALTER TABLE dataset_frame_empirical RENAME TO dataset_frame_gt;

DROP TABLE dataset_frame_scanline_info_empirical;
CREATE TABLE dataset_frame_scanline_gt
(
    id integer PRIMARY KEY AUTOINCREMENT,
    dataset_frame_id integer NOT NULL REFERENCES dataset_frame (id),
    laser_id integer NOT NULL REFERENCES dataset_laser_gt (id),
    scanline_idx integer NOT NULL,

    UNIQUE (dataset_frame_id, laser_id),
    UNIQUE (dataset_frame_id, scanline_idx)
);
CREATE INDEX dataset_frame_scanline_gt_dataset_frame_id_idx ON dataset_frame_scanline_gt (dataset_frame_id);
CREATE INDEX dataset_frame_scanline_gt_dataset_frame_id_scanline_idx ON dataset_frame_scanline_gt (dataset_frame_id, laser_id);
