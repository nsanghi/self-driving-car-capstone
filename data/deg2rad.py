import csv
import shutil

FILE   = 'wp_yaw_const.txt'
FIELDS = ['x', 'y', 'z', 'yaw']

original_file = FILE + '.original'

shutil.move(FILE, original_file)

with open(original_file, 'r') as csv_in, open(FILE, 'w') as csv_out:
    reader = csv.DictReader(csv_in, fieldnames=FIELDS)
    writer = csv.DictWriter(csv_out, fieldnames=FIELDS)
    for row in reader:
        new_row = {
                   'x': row['x'],
                   'y': row['y'],
                   'z': row['z'],
                   'yaw': str(float(row['yaw']) * 0.0174533)
                  }
        writer.writerow(new_row) 
