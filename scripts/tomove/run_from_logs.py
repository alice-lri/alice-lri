import re
import subprocess
import concurrent.futures

base_path_logs = "/mnt/netapp1/Store_usccisss/datasets_lidar/durlar"
base_path_local = "../../../Datasets/LiDAR/durlar"

# Read the log file
with open('out.log', 'r') as file:
    lines = file.readlines()

# Extract paths
pattern = re.compile(r'.*Cloud path: (.*) \(.*\)')
paths = [match.group(1) for line in lines if (match := pattern.match(line))]
paths = [paths.replace(base_path_logs, base_path_local) for paths in paths]

# Reverse the order of paths
paths.reverse()

def process_path(path):
    result = subprocess.run(['../cmake-build-release/examples/examples', path])
    if result.returncode != 0:
        raise SystemExit(f"Command failed for path {path}")

N = 16
with concurrent.futures.ThreadPoolExecutor(max_workers=N) as executor:
    executor.map(process_path, paths)