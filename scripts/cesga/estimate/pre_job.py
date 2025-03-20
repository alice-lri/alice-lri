import argparse
import sqlite3

parser = argparse.ArgumentParser(description='Prepare the DB for a new experiment.')
parser.add_argument('db_path', type=str, help='Path to the SQLite database')
args = parser.parse_args()

conn = sqlite3.connect(args.db_path)
cur = conn.cursor()

cur.execute("INSERT INTO experiment(timestamp) VALUES (datetime('now', 'localtime', 'subsec'))")

conn.commit()
conn.close()
