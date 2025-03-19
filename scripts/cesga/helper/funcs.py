import os
import shutil

def backup_db(merged_db_path):
    base_backup_db_path = merged_db_path + '.bak'
    backup_db_path = base_backup_db_path

    if os.path.exists(base_backup_db_path):
        index = 1
        while os.path.exists(f'{base_backup_db_path}.{index}'):
            index += 1
        backup_db_path = f'{base_backup_db_path}.{index}'

    if os.path.exists(merged_db_path):
        shutil.copy(merged_db_path, backup_db_path)
