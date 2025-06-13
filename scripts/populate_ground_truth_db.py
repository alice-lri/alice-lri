import numpy as np
import sqlite3
import argparse
import os


def load_binary(file_path):
    data = np.fromfile(file_path, dtype=np.float32)
    data = data.reshape((-1, 4))  # Assuming the format is x, y, z, intensity
    points = data[:, :3]
    intensity = data[:, 3]

    return points, intensity


def calculate_phi(points):
    distances_xy = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2)
    return np.arctan2(points[:, 2], distances_xy)


def calculate_range(points):
    return np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2 + points[:, 2] ** 2)


def get_kitti_constants():
    v_offsets = [0.10517547, 0.10582327, 0.10652189, 0.10736023, 0.10795722, 0.10849071, 0.10912581, 0.10990064,
                 0.11056115, 0.11125976, 0.11185676, 0.11236484, 0.11299995, 0.11357154, 0.11428286, 0.11493066,
                 0.11568009, 0.11612466, 0.11673436, 0.11728055, 0.11796646, 0.11860156, 0.11924937, 0.11975745,
                 0.12030364, 0.12081173, 0.12139602, 0.12212004, 0.12266623, 0.1231235, 0.12366969, 0.12427939,
                 0.19555548, 0.19606993, 0.19639328, 0.19687834, 0.19737808, 0.19774553, 0.1981571, 0.19859804,
                 0.19906839, 0.19950935, 0.19996502, 0.20039125, 0.20080282, 0.20127317, 0.20162594, 0.20212568,
                 0.20250784, 0.20296349, 0.20337505, 0.20380131, 0.20427166, 0.20469791, 0.20506536, 0.20556513,
                 0.20600607, 0.20646173, 0.20679979, 0.20729954, 0.20766701, 0.20821083, 0.20854891, 0.20900455]

    v_angles = np.deg2rad(
        [-24.999201, -24.506605, -23.971016, -23.322384, -22.856577, -22.437605, -21.935509, -21.318125, -20.787691,
         -20.222572, -19.736372, -19.320236, -18.797075, -18.323431, -17.73037, -17.186821, -16.55401, -16.176632,
         -15.656734, -15.188733, -14.598064, -14.048302, -13.484814, -13.040989, -12.562096, -12.115005, -11.598996,
         -10.956945, -10.470731, -10.06249, -9.5735149, -9.0260181, -8.7114143, -8.3104696, -8.0580254, -7.6787701,
         -7.287312, -6.9990368, -6.675746, -6.3288889, -5.958395, -5.6106009, -5.2507782, -4.9137921, -4.5881028,
         -4.2155242, -3.935853, -3.5393341, -3.235882, -2.8738379, -2.546633, -2.207566, -1.833245, -1.49388, -1.201239,
         -0.80315, -0.45182899, -0.088762, 0.180617, 0.57880998, 0.871566, 1.304757, 1.573966, 1.9367])

    h_offsets = [0.026, -0.026] * (64 // 2)
    h_resolutions = [4000] * 64

    return v_offsets, v_angles, h_offsets, h_resolutions


# TODO these values are based off emprical medians, not actual constants, change when constants are available
def get_durlar_constants():
    v_offsets = [0.03948696519285808, 0.039488849176797305, 0.03947743833311064, 0.03944817174837266,
                 0.039434128520796766, 0.03943182893416684, 0.03941507314308524, 0.03938160327556467,
                 0.03936390616187149, 0.039354797841253064, 0.0393316086671333, 0.03929102923218217, 0.0392655735016471,
                 0.03924719623294812, 0.039219440991232854, 0.03917949072569306, 0.03915137649611175,
                 0.0391327387185637, 0.039104962843056174, 0.03906199589061276, 0.03902617524283428,
                 0.03899959817310836, 0.03896321004973497, 0.038915773123923, 0.03887529975624082, 0.03884483887479275,
                 0.03880278197141872, 0.038751157675469716, 0.03870346011154719, 0.03866530294509572,
                 0.03861793777702698, 0.038561471289168814, 0.03850541915006385, 0.03846188194100361,
                 0.038407533534441006, 0.03834577216986177, 0.038284893219851804, 0.03823395831313981,
                 0.03817333025795139, 0.038106836522483326, 0.03804021449874462, 0.037979520453409604,
                 0.037914374512422355, 0.03784287146674743, 0.037771706454004454, 0.03770433480915914,
                 0.03763620139040451, 0.03755884810803181, 0.03747978896149212, 0.037408876456993846,
                 0.03733094557274244, 0.0372512051516936, 0.03716396609848537, 0.03708682860648866, 0.03700343390062763,
                 0.03691921869547509, 0.03683087343964185, 0.03674497993742076, 0.0366586401317479,
                 0.036567453959205756, 0.036468211438980006, 0.03637852582894462, 0.0362841392303067,
                 0.036188382699632746, 0.03608874949066987, 0.03599005799092245, 0.035890242900494244,
                 0.03579000191098128, 0.03568186526804027, 0.03557681089632159, 0.03547461320174953,
                 0.035369952892085506, 0.03525925957274129, 0.03515113230337781, 0.035037501582522446,
                 0.03493168892730233, 0.03481560183444947, 0.034698194514806324, 0.03458249985976, 0.034469217697899075,
                 0.03435113910905438, 0.0342311062720041, 0.03411379159800083, 0.03399637428158503, 0.0338697465914007,
                 0.033743885685079016, 0.033621681739815876, 0.03350380746083332, 0.03337204550001015,
                 0.0332405180697142, 0.033113548114551916, 0.032991839963211665, 0.03285501399816425,
                 0.03272172542016658, 0.03259009000378235, 0.03246854309106406, 0.03233073979126425,
                 0.03219618162237319, 0.03205606016998667, 0.03193504592601407, 0.031792678590748885,
                 0.031644799708739346, 0.03151250804831368, 0.031384081442648495, 0.03124555380296888,
                 0.031092583219191144, 0.03095628414182516, 0.03082455595212949, 0.03068189206470507,
                 0.030528216953340126, 0.03038801881099294, 0.030257517837311898, 0.030110854009030602,
                 0.029956886071873315, 0.02981287962126362, 0.029684041869043727, 0.029528821752746848,
                 0.029374975419842353, 0.029227302648357698, 0.029100349283961643, 0.028955291355218,
                 0.028792551497257166, 0.02864135197410625, 0.028511817499632303, 0.02836820176033289,
                 0.028206070568577254, 0.028056317953952102, 0.02792438641717128]

    v_angles = np.deg2rad(
        [-21.220731486455456, -20.920675954550184, -20.61065535358159, -20.310580414198217, -19.970576506713883,
         -19.680540057145606, -19.37051364206931, -19.06044992836965, -18.72046141694171, -18.42041476526568,
         -18.11037037289559, -17.80026732411351, -17.450261264142117, -17.140184273523147, -16.83014663931392,
         -16.500126114101693, -16.160136817127523, -15.84013384878121, -15.530146261061295, -15.200139225320177,
         -14.850137547833501, -14.53011367055886, -14.210101486027554, -13.88009430806334, -13.52010987719631,
         -13.200105294357632, -12.870099763545085, -12.54009351739135, -12.180095991377351, -11.85008839203626,
         -11.520082505240078, -11.190073127648889, -10.820067219747047, -10.500055675830959, -10.160050067455044,
         -9.820046976804278, -9.450049021043995, -9.12004117124245, -8.780031526265807, -8.440028460763253,
         -8.08002050863734, -7.730014822014435, -7.390012212698219, -7.050007933459382, -6.690007744049054,
         -6.34000306720316, -6.010002558512154, -5.660002294110634, -5.2900021936000785, -4.950001231968536,
         -4.60000067348, -4.260000603136574, -3.88000059416245, -3.5400005060120083, -3.1900004573833747,
         -2.8500004125253424, -2.4900004184391835, -2.1400003501363893, -1.8000003079847389, -1.4500002729157426,
         -1.070000283510297, -0.7300002524079987, -0.3800002292314918, -0.030000218112061692, 0.32999976231334294,
         0.6799997785882329, 1.0299997862145105, 1.3799998034570633, 1.7499997798210911, 2.0999997872182745,
         2.4399997967697584, 2.7899998069896705, 3.1499997826184023, 3.4899997867320884, 3.849999802643356,
         4.189999813051708, 4.549999789654099, 4.899999787787802, 5.249999796606842, 5.599999813807512,
         5.949999795160293, 6.289999785266355, 6.629999793499865, 6.979999810405912, 7.339999798381425,
         7.679999787973986, 8.019999793716863, 8.359999816572081, 8.719999806539967, 9.059999788983506,
         9.399999796740584, 9.73999982237913, 10.099999807343048, 10.42999978593035, 10.76999978559444,
         11.099999810445107, 11.44999980435092, 11.769999777525504, 12.119999776069863, 12.439999810262805,
         12.789999801449948, 13.1299997755205, 13.449999773320812, 13.77999981171264, 14.10999980907667,
         14.449999773157863, 14.76999977721349, 15.099999815021684, 15.429999811888377, 15.75999976712763,
         16.079999760161563, 16.39999981128538, 16.7299998124133, 17.049999765952574, 17.369999762958365,
         17.679999821835448, 18.019999814920563, 18.329999761262417, 18.649999764875094, 18.949999826377717,
         19.259999823833308, 19.579999761238724, 19.899999760228063, 20.199999824768213, 20.499999824500232,
         20.809999752979724, 21.119999751535186, 21.419999821646748])

    h_offsets = [-0.0011603541562580815, -0.0003889327769041652, 0.0003889328392576074, 0.0011631054103431876] * (
            128 // 4)
    h_resolutions = [2048] * 128

    return v_offsets, v_angles, h_offsets, h_resolutions


def compute_ground_truth(points, v_angles, v_offsets, h_offsets, h_resolutions, threshold=5e-4):
    assert np.all(np.diff(v_angles) > 0), "v_angles are not in ascending order"
    assert len(v_offsets) == len(v_angles), "v_offsets and v_angles have different lengths"

    phis = calculate_phi(points)
    ranges = calculate_range(points)
    scanlines_ids = np.full(len(points), -1, dtype=int)

    laser_idx = 0
    while laser_idx < len(v_offsets):
        v = v_offsets[laser_idx]
        phi_correction = np.arcsin(v / ranges)

        # find indices where correction was successful and phi is equal to the corresponding v_angle +- 1e-3
        idx = np.where(np.abs(phis - phi_correction - v_angles[laser_idx]) < threshold)[0]

        if len(idx) == 0:
            laser_idx += 1
            continue

        assert np.all(scanlines_ids[idx] == -1), "Some points were assigned to multiple scanlines"

        # assign scanline id to the corresponding indices
        scanlines_ids[idx] = laser_idx
        laser_idx += 1

    assert np.all(scanlines_ids != -1), "Some points were not assigned to any scanline"

    result = {
        'points_count': len(points),
        'scanlines_count': np.unique(scanlines_ids).shape[0],
        'scanlines': []
    }

    for laser_idx in np.unique(scanlines_ids):
        v_offset = v_offsets[laser_idx]
        v_angle = v_angles[laser_idx]
        h_offset = h_offsets[laser_idx]
        h_resolution = h_resolutions[laser_idx]

        points_count = int(np.sum(scanlines_ids == laser_idx))

        result['scanlines'].append({
            'laser_idx': int(laser_idx),
            'v_offset': v_offset,
            'v_angle': v_angle,
            'h_offset': h_offset,
            'h_resolution': h_resolution,
            'points_count': points_count
        })

    return scanlines_ids, result


def get_sensor_properties(dataset_name):
    if dataset_name == 'kitti':
        return get_kitti_constants()
    elif dataset_name == 'durlar':
        return get_durlar_constants()
    else:
        raise ValueError(f"Unknown dataset name: {dataset_name}")


def compute_ground_truth_from_file(input_path, dataset_name):
    points, _ = load_binary(input_path)
    points = points[calculate_range(points) > 0]

    v_offsets, v_angles, h_offsets, h_resolutions = get_sensor_properties(dataset_name)
    scanlines_ids, result = compute_ground_truth(points, v_angles, v_offsets, h_offsets, h_resolutions)

    return result


def store_ground_truth(ground_truth, frame_id, db_cursor):
    db_cursor.execute('''
        INSERT OR IGNORE INTO dataset_frame_empirical(dataset_frame_id, points_count, scanlines_count)
        VALUES (?, ?, ?)
    ''', (frame_id, ground_truth['points_count'], ground_truth['scanlines_count']))

    db_cursor.executemany('''
        INSERT OR IGNORE INTO dataset_frame_scanline_info_empirical(dataset_frame_id, scanline_idx, laser_idx, points_count,
                                                                    vertical_offset, vertical_angle, horizontal_offset,
                                                                    horizontal_resolution)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?)
    ''', [(frame_id, idx, scanline['laser_idx'], scanline['points_count'], scanline['v_offset'], scanline['v_angle'],
           scanline['h_offset'], scanline['h_resolution']) for idx, scanline in enumerate(ground_truth['scanlines'])])


def get_frames_for_process(db_path, process_id, total_processes):
    """Get frames that should be processed by this process based on ID."""

    conn = sqlite3.connect(db_path)
    cur = conn.cursor()

    # Get all dataset names with their IDs
    cur.execute("SELECT id, name FROM dataset")
    datasets = {row[0]: row[1] for row in cur.fetchall()}

    # Get frames assigned to this process based on ID modulo
    cur.execute(
        "SELECT id, dataset_id, relative_path FROM dataset_frame WHERE id % ? = ?",
        (total_processes, process_id)
    )

    frames = []
    for frame_id, dataset_id, relative_path in cur.fetchall():
        dataset_name = datasets.get(dataset_id)
        if dataset_name:
            frames.append({
                'id': frame_id,
                'dataset_id': dataset_id,
                'dataset_name': dataset_name,
                'relative_path': relative_path
            })

    conn.close()
    return frames


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Populate ground truth database in parallel.')
    parser.add_argument('process_id', type=int, help='ID of the current process (0-indexed)')
    parser.add_argument('total_processes', type=int, help='Total number of processes')
    parser.add_argument('--db_path', type=str, required=True, help='Path to the SQLite database')
    parser.add_argument('--kitti_root', type=str, required=True, help='Root path for KITTI dataset')
    parser.add_argument('--durlar_root', type=str, required=True, help='Root path for DurLAR dataset')
    args = parser.parse_args()

    assert os.path.exists(args.db_path), f"Database path does not exist: {args.db_path}"
    assert os.path.exists(args.kitti_root), f"KITTI root path does not exist: {args.kitti_root}"
    assert os.path.exists(args.durlar_root), f"DurLAR root path does not exist: {args.durlar_root}"

    process_id = args.process_id
    total_processes = args.total_processes

    # Create a mapping of dataset names to their root paths
    dataset_roots = {
        'kitti': args.kitti_root,
        'durlar': args.durlar_root
    }

    # Get frames to process for this worker
    frames = get_frames_for_process(args.db_path, process_id, total_processes)
    print(f"Process {process_id}/{total_processes} - Assigned {len(frames)} frames")

    # Connect to the database for writing results
    conn = sqlite3.connect(args.db_path)
    cur = conn.cursor()

    for i, frame in enumerate(frames):
        # Build the full path to the file
        full_path = os.path.join(dataset_roots[frame['dataset_name']], frame['relative_path'])

        # Process the file
        ground_truth = compute_ground_truth_from_file(full_path, frame['dataset_name'])
        store_ground_truth(ground_truth, frame['id'], cur)

        print(f"Process {process_id}/{total_processes} - Processed {i + 1}/{len(frames)} frames")

    # Only commit at the end
    conn.commit()
    conn.close()

    print(f"Process {process_id}/{total_processes} - Finished all {len(frames)} frames successfully")
