import sys

import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import seaborn as sns
import os

def draw_square(x, y, w_mar, h_mar, color):
    plt.plot([x - w_mar, x + w_mar, x + w_mar, x - w_mar, x - w_mar],
             [y - h_mar, y - h_mar, y + h_mar, y + h_mar, y - h_mar], color=color)

def plt_init(mode):
    if mode == 'interactive':
        matplotlib.use('Qt5Agg')


def plt_show(mode, root_folder, name):
    if mode == 'inline':
        plt.show()
    elif mode == 'interactive':
        plt.show(block=True)
        matplotlib.use('Qt5Agg')
    elif mode == 'files':
        plt.savefig(f"{root_folder}/{name}.png")
        plt.close()

def plot_debug_info(iteration, ranges, phis, scanline_lower_limit, scanline_upper_limit, accumulator,
                    points_in_scanline_mask, unassigned_mask, offsets, angle_uncorrected_range, picked_offset,
                    picked_angle, scanlines_attributes, uncertainty, step_by_step_debug_info_mode, root_folder,
                    files_prefix='', plot_accumulator=True):
    plt_init(step_by_step_debug_info_mode)
    plt.figure(figsize=(6, 24))

    ranges_sample = np.linspace(np.min(ranges), np.max(ranges), 1000)
    for id, scanline in scanlines_attributes.items():
        uncertainty = scanline['uncertainty']
        angle_values = np.arcsin(scanline['offset'] / ranges_sample) + scanline['angle']
        line_color = 'lightcoral' if uncertainty == np.inf else 'lightgreen'
        plt.plot(1 / ranges_sample, angle_values, color=line_color, linewidth=0.5, linestyle='dashed')
        plt.text(1 / ranges_sample[0], angle_values[0], f'{uncertainty:.2f}', fontsize=8, color='black', va='center')

    if scanline_upper_limit is not None and scanline_lower_limit is not None and len(scanline_upper_limit) > 0 and len(scanline_lower_limit) > 0:
        plt.scatter(1 / ranges, scanline_lower_limit, color='yellow', s=0.1)
        plt.scatter(1 / ranges, scanline_upper_limit, color='cyan', s=0.1)

    plt.scatter(1 / ranges[~unassigned_mask], phis[~unassigned_mask], s=0.4, color='blue')

    scanline_points_color = 'orange' if uncertainty == np.inf else 'green'
    plt.scatter(1 / ranges[points_in_scanline_mask], phis[points_in_scanline_mask], s=0.4, color=scanline_points_color)
    plt.scatter(1 / ranges[unassigned_mask & ~points_in_scanline_mask], phis[unassigned_mask & ~points_in_scanline_mask], s=0.4, color='red')
    plt_show(step_by_step_debug_info_mode, root_folder, f"{files_prefix}scanlines_{iteration}")

    if plot_accumulator:
        plt_init(step_by_step_debug_info_mode)
        plt.figure(figsize=(24, 16))
        plt.subplot(2, 1, 1)
        plt.imshow(accumulator, aspect='auto',
                   extent=(offsets[0], offsets[-1], angle_uncorrected_range[0], angle_uncorrected_range[-1]),
                   interpolation='nearest',
                   origin='lower')

        # draw a green square in the area of the maximum in the hough accumulator
        width_margin_pixels = 10
        height_margin_pixels = 10
        zoom_offset_margin = width_margin_pixels * (offsets[1] - [offsets[0]])
        zoom_angle_margin = height_margin_pixels * (angle_uncorrected_range[1] - angle_uncorrected_range[0])
        draw_square(picked_offset, picked_angle, zoom_offset_margin, zoom_angle_margin, 'green')

        plt.colorbar()
        plt.xlabel('Offset')
        plt.ylabel('Angle (radians)')
        plt.title('Hough Accumulator')

        # plot again the imshow but zoomed in the area of the maximum
        max_angle_index = np.argwhere(angle_uncorrected_range == picked_angle)
        max_offset_index = np.argwhere(offsets == picked_offset)
        accumulator_cropped = accumulator[
                              np.max([0, max_angle_index[0, 0] - height_margin_pixels])
                              :np.min([accumulator.shape[0], max_angle_index[0, 0] + height_margin_pixels + 1]),
                              np.max([0, max_offset_index[0, 0] - width_margin_pixels])
                              :np.min([accumulator.shape[1], max_offset_index[0, 0] + width_margin_pixels + 1])
                              ]

        offset_min_cropped = picked_offset - zoom_offset_margin
        offset_max_cropped = picked_offset + zoom_offset_margin
        angle_uncorrected_min_cropped = picked_angle - zoom_angle_margin
        angle_uncorrected_max_cropped = picked_angle + zoom_angle_margin

        plt.subplot(2, 1, 2)
        # plt.imshow(accumulator_cropped, aspect='auto', extent=(offset_min_cropped, offset_max_cropped, angle_uncorrected_min_cropped, angle_uncorrected_max_cropped), interpolation='nearest', origin='lower')
        sns.heatmap(
            accumulator_cropped, annot=True, fmt=".2f", cmap="magma",
            cbar=True, cbar_kws={'label': 'Votes'},
            xticklabels=np.round(np.linspace(offset_min_cropped, offset_max_cropped, 2 * width_margin_pixels + 1), 6),
            yticklabels=np.round(
                np.linspace(angle_uncorrected_min_cropped, angle_uncorrected_max_cropped, 2 * height_margin_pixels + 1),
                6),

        )
        draw_square(width_margin_pixels + 0.5, height_margin_pixels + 0.5, 0.5, 0.5, 'green')
        plt.xlabel('Offset')
        plt.ylabel('Angle (radians)')
        # invert y-axis
        plt.gca().invert_yaxis()
        plt.title('Hough Accumulator Zoomed')
        plt_show(step_by_step_debug_info_mode, root_folder, f"{files_prefix}accumulator_{iteration}")

def read_binary_file(file_path):
    return np.fromfile(file_path, dtype=np.float64)

def read_binary_file_bool(file_path):
    return np.fromfile(file_path, dtype=np.byte).astype(bool)

def main(folder, prefix, iteration, offset, angle, uncertainty):
    ranges = read_binary_file(os.path.join(folder, 'ranges.bin'))
    phis = read_binary_file(os.path.join(folder, 'phis.bin'))
    scanline_lower_limit = read_binary_file(os.path.join(folder, 'scanline_lower_limit.bin'))
    scanline_upper_limit = read_binary_file(os.path.join(folder, 'scanline_upper_limit.bin'))
    points_in_scanline_mask = read_binary_file_bool(os.path.join(folder, 'points_in_scanline_mask.bin'))
    unassigned_mask = read_binary_file_bool(os.path.join(folder, 'unassigned_mask.bin'))
    picked_offset = offset
    picked_angle = angle
    scanlines_attributes = {}  # Load or define as needed
    step_by_step_debug_info_mode = 'files'  # Adjust as needed

    plot_debug_info(iteration, ranges, phis, scanline_lower_limit, scanline_upper_limit, None,
                    points_in_scanline_mask, unassigned_mask, None, None, picked_offset,
                    picked_angle, scanlines_attributes, uncertainty, step_by_step_debug_info_mode, folder,
                    files_prefix=prefix, plot_accumulator=False)

if __name__ == '__main__':
    if len(sys.argv) < 7:
        print(f"Usage: {sys.argv[0]} <folder> <prefix> <iteration> <offset> <angle> <uncertainty>")
        sys.exit(1)


    folder = sys.argv[1]
    prefix = sys.argv[2]
    iteration = int(sys.argv[3])
    offset = float(sys.argv[4])
    angle = float(sys.argv[5])
    uncertainty = float(sys.argv[6])

    main(folder, prefix, iteration, offset, angle, uncertainty)
