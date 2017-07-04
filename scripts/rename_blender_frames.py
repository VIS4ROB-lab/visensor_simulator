import numpy as np
import argparse
import os
import sys
import csv

#setup the argument list
parser = argparse.ArgumentParser(description='Rename images from blender')
parser.add_argument('--folder',  metavar='folder', help='blender result folder')

#print help if no argument is specified
if len(sys.argv)<2:
    parser.print_help()
    sys.exit(0)

#parse the args
parsed = parser.parse_args()

with open(os.path.join(parsed.folder, 'blender_id_time.csv'),'r') as csvfile:
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
