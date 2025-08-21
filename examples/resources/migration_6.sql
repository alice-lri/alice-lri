ALTER TABLE ri_frame_result
    ADD COLUMN reconstructed_to_original_rmse real NOT NULL DEFAULT 0;

ALTER TABLE ri_frame_result
    ADD COLUMN original_to_reconstructed_rmse real NOT NULL DEFAULT 0;

ALTER TABLE compression_frame_result
    ADD COLUMN naive_to_original_rmse real NOT NULL DEFAULT 0;

ALTER TABLE compression_frame_result
    ADD COLUMN original_to_naive_rmse real NOT NULL DEFAULT 0;

ALTER TABLE compression_frame_result
    ADD COLUMN accurate_to_original_rmse real NOT NULL DEFAULT 0;

ALTER TABLE compression_frame_result
    ADD COLUMN original_to_accurate_rmse real NOT NULL DEFAULT 0;
