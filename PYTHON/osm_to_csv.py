import osmnx as ox
import csv

features = ox.features.features_from_bbox(37.6636, 37.6218, 34.6771, 34.7412, {'highway': True})
road_tuples = [ ]
for i in range(len(features['geometry'])):
    if(features['geometry'][i].type == 'LineString'):
        road_tuples.append([])
        for j in range( len(features['geometry'][i].coords) ):
            road_tuples[i].append(features['geometry'][i].coords[j])
    
print(len(road_tuples))
print(len(road_tuples[0]))


for k in range(len(road_tuples)):
    csv_file = str(features['ref'][k]) +  '___'+ str(k) + '.csv'

    csvfile = open(csv_file, 'w', newline='')

    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['Longitude', 'Latitude'])
    #print(len(road_tuples[k]))
    for row in road_tuples[k]:
        #print(row)
        csv_writer.writerow(row)
    csvfile.close()

