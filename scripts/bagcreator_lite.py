#!/usr/bin/env python
# -*- coding: utf-8 -*-
#License (BSD)
#Copyright (c) 2018, Lucas Teixeira, Vision for Robotics Lab, ETH Zurich, Switzerland
#Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder, Autonomous Systems Lab, ETH Zurich, Switzerland
#Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland
#most code in this file is from kalibr_bagcreater -ethz-asl/kalibr
#we only change the input structure to VISim project

import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
import ImageFile
import time, sys, os
import argparse
import cv2
import numpy as np
import csv
import json


## Dataformat for the JSON project

class VISimCamera():
    cam_name = "cam_default"
    focal_lenght = 455
    frequency_reduction_factor = 99 # if 10 then it is 10 times SLOWER than the imu that runs at 200Hz
    height = 480
    width = 752
    transform_ImuCamera = [0, -1, 0, 0, #frontal camera in ros standard
                          -1, 0, 0, 0,
                          0, 0, -1, 0,
                          0, 0, 0, 1] 
    #todo add param with the delay between the imu and image
    
    
    
    def toJSON(self):
        result = {}
        result["cam_name"] = self.cam_name
        result["focal_lenght"] = self.focal_lenght
        result["frequency_reduction_factor"] = self.frequency_reduction_factor
        result["height"] = self.height
        result["width"] = self.width
        result["T_SC"] = self.transform_ImuCamera
        return result
    
    def fromJSON(self, json_dict):
        try:
            self.cam_name = json_dict["cam_name"]
            self.focal_lenght = json_dict["focal_lenght"]
            self.frequency_reduction_factor = json_dict["frequency_reduction_factor"]
            self.height = json_dict["height"]
            self.width = json_dict["width"]
            self.transform_ImuCamera = json_dict["T_SC"]
        except KeyError as e:
            print ('KeyError - cam_id {} reason {}'.format(self.cam_name , str(e)))
            return False
        return True

class VISimProject():
    
    cameras=[]
    name="" 
    
    def camerasToJSON(self):
        result = []
        for cam in self.cameras:
            result.append(cam.toJSON())
        return result
        
    def camerasFromJSON(self, cameraJSONs):
        result = []
        for iCamJSON in cameraJSONs:
            currCam = VISimCamera()
            if(currCam.fromJSON(iCamJSON)):            
                result.append(currCam)
            else:
                return False
                
        self.cameras = result
        
        return True
    
    def toJSON(self):
        result = {}
        result["name"] = self.name
        result["cameras"] = self.camerasToJSON()
        return result
    
    def fromJSON(self, json_dict):
        try:
            self.name = json_dict["name"]
        except KeyError as e:
            print ("KeyError - in project reason {}".format(str(e)))
            
        if(self.camerasFromJSON(json_dict["cameras"]) == False):
            return False
        return True



def getImageFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    image_files = list()
    timestamps = list()
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg']:
                    image_files.append( f ) 
                    
    #sort by timestamp
    image_files = sorted( image_files)
   # image_files = [file for file in sort_list]
    return image_files

def getCamFoldersFromDir(dir):
    '''Generates a list of all folders that start with cam e.g. cam0'''
    cam_folders = list()
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for folder in folders:
                if folder[-4:] == "_rgbd":
                    cam_folders.append((folder[-4:],folder))
    return cam_folders


def loadImageToRosMsg(timestamp,camdir,filename):
    filepath = os.path.join(camdir, filename)
    image_np = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
    
    rosimage = Image()
    rosimage.header.stamp = timestamp
    rosimage.height = image_np.shape[0]
    rosimage.width = image_np.shape[1]
    rosimage.step = rosimage.width  #only with mono8! (step = width * byteperpixel * numChannels)
    rosimage.encoding = "mono8"
    rosimage.data = image_np.tostring()
    
    return rosimage

def createImuMessge(timestamp_int, omega, alpha):
    timestamp_nsecs = str(timestamp_int)
    timestamp = rospy.Time( int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) )
    
    rosimu = Imu()
    rosimu.header.stamp = timestamp
    rosimu.angular_velocity.x = float(omega[0])
    rosimu.angular_velocity.y = float(omega[1])
    rosimu.angular_velocity.z = float(omega[2])
    rosimu.linear_acceleration.x = float(alpha[0])
    rosimu.linear_acceleration.y = float(alpha[1])
    rosimu.linear_acceleration.z = float(alpha[2])
    
    return rosimu, timestamp
    
def lixo():
    #write imu data 
    imufiles = getImuCsvFiles(imu_filepath)
    for imufile in imufiles:
        topic = os.path.splitext(os.path.basename(imufile))[0]
        with open(imufile, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            headers = next(reader, None)
            for row in reader:
                imumsg, timestamp = createImuMessge(row[0], row[1:4], row[4:7])
                bag.write("/{0}".format(topic), imumsg, timestamp)
                
                 #write images
    camfolders = getCamFoldersFromDir(parsed.folder)
    for camfolder in camfolders:
        camdir = parsed.folder + "/{0}".format(camfolder)
        image_files = getImageFilesFromDir(camdir)
        for image_filename in image_files:
            image_msg, timestamp = loadImageToRosMsg(image_filename)
            bag.write("/{0}/image_raw".format(camfolder), image_msg, timestamp)

    
    


if __name__ == "__main__":

    #setup the argument list
    parser = argparse.ArgumentParser(description='Create a ROS bag using the images and imu data.')
    parser.add_argument('--folder',  metavar='folder', nargs='?', help='VISim Project folder')
    parser.add_argument('--output-bag', metavar='output_bag',  default="output.bag", help='ROS bag file %(default)s')

    #print help if no argument is specified
    if len(sys.argv)<2:
        parser.print_help()
        sys.exit(0)

    #parse the args
    parsed = parser.parse_args()
    proj_filepath = os.path.join(parsed.folder, 'visim_project.json')
        
    #create the bag
    try:
        with open(proj_filepath) as json_data:
            try:
                project_data = json.load(json_data)
            except ValueError as e:
                print('Problem with the JSON file parsing. '+ str(e))                
                sys.exit()
                
        bag = rosbag.Bag(parsed.output_bag, 'w')
       
        visim_json_project = VISimProject()
        if (visim_json_project.fromJSON(project_data) == False):            
             print( 'Problem with the JSON file parsing. Look in the terminal')
             sys.exit()
        
        imu_filepath = os.path.join(parsed.folder, 'output/1_Rotors/imu_data.csv')
        
        print("okkkk " + visim_json_project.name)
        imu_topic = "/" + visim_json_project.name + "/imu"
        imu_timestamps = list()
        with open(imu_filepath, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            headers = next(reader, None)
            for row in reader:
                imumsg, timestamp = createImuMessge(row[0], row[1:4], row[4:7])
                bag.write("/{0}".format(imu_topic), imumsg, timestamp)
                imu_timestamps.append(timestamp)
       
        for cam_data in visim_json_project.cameras:
            cam_dirpath = os.path.join(parsed.folder, 'output/2_Blender/' + cam_data.cam_name + '_rgbd')
            cam_topic = "/{0}/{1}/image_raw".format(visim_json_project.name,cam_data.cam_name) #"/" + visim_json_project.name + "/" + cam_data.cam_name
            cam_image_files = getImageFilesFromDir(cam_dirpath)
            #image_ids = [ for file in cam_image_files]
            #print(image_ids)
            for image_filename in cam_image_files:
                try:
                    idx = int(image_filename[3:-4])-1
                    timestamp = imu_timestamps[idx]
                    timestamp.nsecs +=1
                    image_msg = loadImageToRosMsg(timestamp,cam_dirpath,image_filename)
                    bag.write(cam_topic, image_msg, timestamp)
                except IndexError as e:
                    print('Index {0} doesnt exist. '.format(idx)+ str(e))
                        
    finally:
        bag.close()
    
