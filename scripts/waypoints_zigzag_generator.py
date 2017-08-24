import numpy as np
import argparse
import os
import sys
import csv

#setup the argument list
parser = argparse.ArgumentParser(description='Generate a zig-zag trajectory')
parser.add_argument('-o','--output',  metavar='output', help='output waypoint file')
parser.add_argument('-w','--width',  metavar='width', help='width')
parser.add_argument('-l','--length',  metavar='length', help='length')
parser.add_argument('-gsd','--ground_sample_distance',  metavar='height',default=0.1, help='ground sample distance (in meters)')
parser.add_argument('-fovh','--field_of_view_horizontal',  metavar='fovh',default=65, help='field of view - horizontal(in degrees)')
parser.add_argument('-img_h','--image_resolution_horizontal',  metavar='cam_h',default=2560, help='image resolution - horizontal( in pixels)')
parser.add_argument('-img_v','--image_resolution_vertical',  metavar='cam_v',default=1920, help='image resolution - vertical( in pixels)')


#print help if no argument is specified
if len(sys.argv)<2:
    parser.print_help()
    sys.exit(0)

#parse the args
parsed = parser.parse_args()

print parsed



with open(parsed.output,'w') as csvfile:
    reader = csv.DictReader(csvfile,fieldnames=['id','time'])
    for row in reader:
    	old_name = os.path.join(parsed.folder,'cam0','rgb_'+row['id']+'.png')
    	new_name = os.path.join(parsed.folder,'cam0',row['time']+'.png')
    	print(long(row['time']))
    	#print(new_name)
    	try:
    		os.rename(old_name,new_name)    	
    	except  OSError:
    		print('error on '+old_name)

#creating imu0.csv with imu data
with open(os.path.join(parsed.folder, 'vi_imu.csv'), 'r') as inFile, open(os.path.join(parsed.folder, 'imu0.csv'), 'w') as outFile:
    fieldnames = ['field.header.stamp', 'field.angular_velocity.x', 'field.angular_velocity.y',
                  'field.angular_velocity.z',
                  'field.linear_acceleration.x', 'field.linear_acceleration.y', 'field.linear_acceleration.z']
    writer = csv.DictWriter(outFile, fieldnames=fieldnames, extrasaction='ignore')
#  writer.writerow(['timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z'])
    writer.writerow({'field.header.stamp': 'timestamp', 'field.angular_velocity.x': 'omega_x', 'field.angular_velocity.y': 'omega_y', 'field.angular_velocity.z': 'omega_z',
                    'field.linear_acceleration.x': 'alpha_x', 'field.linear_acceleration.y': 'alpha_y', 'field.linear_acceleration.z': 'alpha_z'})

    #r = csv.reader(inFile)
    for row in csv.DictReader(inFile):
        # writes the reordered rows to the new file
        writer.writerow(row)
