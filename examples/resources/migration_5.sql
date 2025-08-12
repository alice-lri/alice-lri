ALTER TABLE experiment 
    ADD COLUMN use_hough_continuity boolean NOT NULL DEFAULT true;

ALTER TABLE experiment 
    ADD COLUMN use_scanline_conflict_solver boolean NOT NULL DEFAULT true;

ALTER TABLE experiment 
    ADD COLUMN use_vertical_heuristics boolean NOT NULL DEFAULT true;

ALTER TABLE experiment 
    ADD COLUMN use_horizontal_heuristics boolean NOT NULL DEFAULT true;
