import os
import csv

directory = 'denemedataset2'
csv_file = 'combined2.csv'

header = ['Radius', 'Depth', 'BBox_X', 'BBox_Y']

# open the combined csv file for writing and write the header row
with open(csv_file, 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(header)

    # loop over each txt file in the directory and append its data to the csv file
    for filename in os.listdir(directory):
        if filename.endswith('.txt'):
            with open(os.path.join(directory, filename), 'r') as txt_file:
                data = txt_file.read().splitlines()
                writer.writerows([row.split() for row in data])

print('Combined csv file saved successfully!')
