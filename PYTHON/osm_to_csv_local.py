import osmnx as ox
import csv
import os
import sys



lat1 = 33.78653
lat2 = 33.78192
lon1 = -84.38507
lon2 = -84.38232
folder_name = "ismail"

#G = ox.graph_from_point((37.79, -122.41), dist=750, network_type='all')
#ox.plot_graph(G)


#features = ox.features.features_from_bbox(lat1, lat2, lon1, lon2, {'highway': True})
#features = ox.features.features_from_bbox(37.6636, 37.6218, 34.6771, 34.7412, {'highway': True}) #basmakci
#features = ox.features.features_from_bbox(52.357817, 52.310913, 11.996033,12.095047, {'highway': True}) #Germany straight
#features = ox.features.features_from_bbox(51.9941, 51.9848, 13.0050,13.0281, {'highway': True}) #Germany turn
#features = ox.features.features_from_bbox(lat1, lat2, lon1, lon2, {'highway': 'motorway'}) #Germany turn
#features = ox.features.features_from_bbox(lat1, lat2, lon1, lon2, {'building': 'True'})
features = ox.features.features_from_bbox(lat1, lat2, lon1, lon2, {'highway': True})
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
    #csv_file = folder_name +"\\" + str(features['ref'][k]) +  '___'+ str(k) + '.csv'
    csv_file = folder_name + '\\' + '103A' + str(k) + '.csv'
    csvfile = open(csv_file, 'w', newline='')

    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['Longitude', 'Latitude'])
    #print(len(road_tuples[k]))
    for row in road_tuples[k]:
        #print(row)
        csv_writer.writerow(row)
    csvfile.close()

