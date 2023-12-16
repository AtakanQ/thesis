import osmnx as ox
import csv

#features = ox.features.features_from_bbox(lat1, lat2, lon1, lon2, {'highway': True})
#features = ox.features.features_from_bbox(37.6636, 37.6218, 34.6771, 34.7412, {'highway': True}) #basmakci
#features = ox.features.features_from_bbox(52.357817, 52.310913, 11.996033,12.095047, {'highway': True}) #Hohenseeden - Parchen
features = ox.features.features_from_bbox(51.9941, 51.9848, 13.0050,13.0281, {'highway': True}) #Hohenseeden - Parchen
road_tuples = [ ]
road_counter = 0
for i in range(len(features['geometry'])):
    if(features['geometry'][i].type == 'LineString'):
        road_tuples.append([])
        
        for j in range( len(features['geometry'][i].coords) ):
            road_tuples[road_counter].append(features['geometry'][i].coords[j])
        road_counter += 1
            
    
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

