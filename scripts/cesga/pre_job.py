import sqlite3

conn = sqlite3.connect("../../examples/experiments/experiments.sqlite")
cur = conn.cursor()

cur.execute("INSERT INTO experiment(timestamp) VALUES (datetime('now', 'localtime', 'subsec'))")

conn.commit()
conn.close()
