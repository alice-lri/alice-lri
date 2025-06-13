ALTER TABLE intrinsics_result_scanline_info
    ADD COLUMN horizontal_angle_offset real NOT NULL DEFAULT 0;

ALTER TABLE dataset_frame_scanline_info_empirical
    ADD COLUMN horizontal_angle_offset real NOT NULL DEFAULT 0;