import osmnx as ox
import csv
import os
import sys

n = len(sys.argv)
print("Total arguments passed:", n)
if(n != 6):
    print("Wrong number of arguments passed to Python script")
    exit(1)
if(os.path.isdir(sys.argv[5])): #path exists
    print("This path name already exists. Will not attempt to overwrite. (Information inside may be obsolete)")
    exit(1)

lat1 = float(sys.argv[1])
lat2 = float(sys.argv[2])
lon1 = float(sys.argv[3])
lon2 = float(sys.argv[4])
folder_name = sys.argv[5]
#features = ox.features.features_from_bbox(lat1, lat2, lon1, lon2, {'highway': True})
#features = ox.features.features_from_bbox(37.6636, 37.6218, 34.6771, 34.7412, {'highway': True}) #basmakci
#features = ox.features.features_from_bbox(52.357817, 52.310913, 11.996033,12.095047, {'highway': True}) #Germany straight
#features = ox.features.features_from_bbox(51.9941, 51.9848, 13.0050,13.0281, {'highway': True}) #Germany turn
features = ox.features.features_from_bbox(lat1, lat2, lon1, lon2, {'highway': 'motorway'}) #Germany turn
road_tuples = [ ]
road_counter = 0
for i in range(len(features['geometry'])):
    if(features['geometry'][i].geom_type == 'LineString'):
        road_tuples.append([])
        
        for j in range( len(features['geometry'][i].coords) ):
            road_tuples[road_counter].append(features['geometry'][i].coords[j])
        road_counter += 1
            
os.mkdir(folder_name)
for k in range(len(road_tuples)):
    csv_file = folder_name +"\\" + str(features['ref'][k]) +  '___'+ str(k) + '.csv'

    csvfile = open(csv_file, 'w', newline='')

    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['Longitude', 'Latitude'])
    #print(len(road_tuples[k]))
    for row in road_tuples[k]:
        #print(row)
        csv_writer.writerow(row)
    csvfile.close()

