#!/usr/bin/env python
# -*- coding: utf-8 -*-
#License (BSD)
#Copyright (c) 2018, Lucas Teixeira, Vision for Robotics Lab, ETH Zurich, Switzerland
#Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder, Autonomous Systems Lab, ETH Zurich, Switzerland
#Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland
#most code in this file is from kalibr_bagcreater -ethz-asl/kalibr
#we only change the input structure to VISim project

from __future__ import print_function
import rosbag
import rospy
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import os
import argparse
import cv2
import numpy as np
import csv
import json
import math
import sys
import OpenEXR
import Imath
from PIL import Image as Im





## Dataformat for the JSON project

class VISimCamera():
    cam_name = "cam_default"
    frame_id = "cam_default"
    focal_length = 455
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
        result["frame_id"] = self.frame_id
        result["focal_length"] = self.focal_length
        result["frequency_reduction_factor"] = self.frequency_reduction_factor
        result["height"] = self.height
        result["width"] = self.width
        result["T_SC"] = self.transform_ImuCamera
        return result
    
    def fromJSON(self, json_dict):
        #obligatory parameters
        try:
            self.cam_name = json_dict["cam_name"]
            self.focal_length = json_dict["focal_length"]
            self.frequency_reduction_factor = json_dict["frequency_reduction_factor"]
            self.height = json_dict["height"]
            self.width = json_dict["width"]
            self.transform_ImuCamera = json_dict["T_SC"]
        except KeyError as e:
            print ('KeyError - cam_id {} reason {}'.format(self.cam_name , str(e)))
            return False
            
        #optional frame_id
        try:
            self.frame_id = json_dict["frame_id"]
        except KeyError:
            self.frame_id = self.cam_name
            
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
        result["camfrom __future__ import print_functioneras"] = self.camerasToJSON()
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
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.exr', '.png', '.jpg','.jpeg']:
                    image_files.append( f ) 
                    
    
    image_files = sorted( image_files)    
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

def loadImageWithOpenCV(filepath):
    image_np = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)        
    return image_np

def convertSRGBToRGB(img_str,size):
    img = np.fromstring(img_str, dtype=np.float32)
    img = np.where(img<=0.0031308,
                (img*12.92)*255.0,
                (1.055*(img**(1.0/2.4))-0.055) * 255.0)
    img.shape = (size[1], size[0])

    return Im.fromarray(img,'F').convert("L")
   
    
def loadImageWithOpenEXR(filepath):    
    image_exr = OpenEXR.InputFile(filepath)  
    dw = image_exr.header()['dataWindow']
    size = (dw.max.x - dw.min.x + 1, dw.max.y - dw.min.y + 1)
    precision = Imath.PixelType(Imath.PixelType.FLOAT)
    Z = image_exr.channel('Z', precision)
    image_depth = np.fromstring(Z, dtype=np.float32)
    image_depth[image_depth == np.inf] = 0 # conversion: invalid depth in the exr is inf and on ros depth image is 0
    image_depth.shape = (size[1], size[0])

    r = convertSRGBToRGB(image_exr.channel('R', precision),size)
    g = convertSRGBToRGB(image_exr.channel('G', precision),size)
    b = convertSRGBToRGB(image_exr.channel('B', precision),size)
                
    image_rgb = np.asarray(Im.merge("RGB", [r,g,b]))
    
    return image_rgb, image_depth

def loadImagesToRosMsg(timestamp,camera_definition,camdir,filename):
    filepath = os.path.join(camdir, filename)
    extension = os.path.splitext(filepath)[1]
    image_msgs = list()
    if(extension in ['.png','.jpeg','.jpg']):
        image_rgb_np = loadImageWithOpenCV(filepath)
        image_msgs.append(['image_raw',createCameraRGBMsg(timestamp,image_rgb_np,camera_definition.frame_id)])
        
    elif (extension == '.exr'):
        image_rgb_np, image_depth_np = loadImageWithOpenEXR(filepath)
        image_msgs.append(['image_raw',createCameraRGBMsg(timestamp,image_rgb_np,camera_definition.frame_id)])
        image_msgs.append(['image_depth',createCameraDepthMsg(timestamp,image_depth_np,camera_definition.frame_id)])
        
#    rosimage = Image()
#    rosimage.header.stamp = timestamp
#    rosimage.header.frame_id = camera_definition.frame_id
#    rosimage.height = image_rgb_np.shape[0]
#    rosimage.width = image_rgb_np.shape[1]
#    rosimage.step = rosimage.width  #only with mono8! (step = width * byteperpixel * numChannels)
#    rosimage.encoding = "mono8"
#    rosimage.data = image_rgb_np.tostring()
    
    return image_msgs

def createCameraGrayMsg(timestamp, image_rgb,frame_id):
    rosimage = Image()
    rosimage.header.stamp = timestamp
    rosimage.header.frame_id = frame_id
    rosimage.height = image_rgb.shape[0]
    rosimage.width = image_rgb.shape[1]
    rosimage.step = rosimage.width  # (step = width * byteperpixel * numChannels)
    rosimage.encoding = "mono8"
    rosimage.data = image_rgb.tostring()
    
    return rosimage
    
def createCameraRGBMsg(timestamp, image_rgb,frame_id):
    rosimage = Image()
    rosimage.header.stamp = timestamp
    rosimage.header.frame_id = frame_id
    rosimage.height = image_rgb.shape[0]
    rosimage.width = image_rgb.shape[1]
    rosimage.step = rosimage.width * 1  * 3 #(step = width * byteperpixel * numChannels)
    rosimage.encoding = "rgb8"
    rosimage.data = image_rgb.tostring()
    
    return rosimage
    
def createCameraDepthMsg(timestamp, image_depth,frame_id):
    rosimage = Image()
    rosimage.header.stamp = timestamp
    rosimage.header.frame_id = frame_id
    rosimage.height = image_depth.shape[0]
    rosimage.width = image_depth.shape[1]
    rosimage.step = rosimage.width * 4 * 1   # (step = width * byteperpixel * numChannels)
    rosimage.encoding = "32FC1"
    rosimage.data = image_depth.tostring()
    
    return rosimage
    
#def createCameraGrayMsg(timestamp, image_rgb,frame_id):
#    rosimage = Image()
#    rosimage.header.stamp = timestamp
#    rosimage.header.frame_id = frame_id
#    rosimage.height = image_rgb.shape[0]
#    rosimage.width = image_rgb.shape[1]
#    rosimage.step = rosimage.width  # (step = width * byteperpixel * numChannels)
#    rosimage.encoding = "mono8"
#    rosimage.data = image_rgb.tostring()
#    
#    return rosimage
  
def createCameraInfoMsg(timestamp, camera_definition):
    
    cam_info_msg = CameraInfo()
    cam_info_msg.header.stamp = timestamp
    cam_info_msg.header.frame_id = camera_definition.frame_id
    
    f = camera_definition.focal_length
    center_x = math.floor(camera_definition.width/2.0) + 0.5
    center_y = math.floor(camera_definition.height/2.0) + 0.5
    
    cam_info_msg.width  = camera_definition.width
    cam_info_msg.height = camera_definition.height
    k = [f, 0, center_x,
         0, f, center_y,
         0, 0,    1.0   ]
    cam_info_msg.K = k
    cam_info_msg.P = [k[0], k[1], k[2], 0,
                      k[3], k[4], k[5], 0,
                      k[6], k[7], k[8], 0]
                      
    cam_info_msg.distortion_model = "plumb_bob"
    cam_info_msg.D = [0.0] * 5

    return cam_info_msg

def createImuMessge(timestamp_int, alpha, omega):
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
    
def createGtPoseMessge(timestamp_int, position, orientation):
    timestamp_nsecs = str(timestamp_int)
    timestamp = rospy.Time( int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) )
    
    rospose = PoseStamped()
    rospose.header.stamp = timestamp
    rospose.pose.position.x = float(position[0])
    rospose.pose.position.y = float(position[1])
    rospose.pose.position.z = float(position[2])
    rospose.pose.orientation.x = float(orientation[0])
    rospose.pose.orientation.y = float(orientation[1])
    rospose.pose.orientation.z = float(orientation[2])
    rospose.pose.orientation.w = float(orientation[3])
    
    return rospose, timestamp
    

if __name__ == "__main__":

    #setup the argument list
    parser = argparse.ArgumentParser(description='Create a ROS bag using the images and imu data.')
    parser.add_argument('--project_folder',  metavar='project', nargs='?', help='VISim Project folder')
    parser.add_argument('--output-bag', metavar='output_bag',  help='output ROS bag file (.bag)')
    parser.add_argument('--namespace', metavar='namespace',  default="", help='topic namespace(e.g. firefly)')

    #print help if no argument is specified
    if len(sys.argv)<2:
        parser.print_help()
        sys.exit(0)

    #parse the args
    parsed = parser.parse_args()
    proj_filepath = os.path.join(parsed.project_folder, 'visim_project.json')
    
    if os.path.isfile(parsed.output_bag) :
        print('Error: the output file already exists!')
        sys.exit()
        
    #create the bag
    try:
        with open(proj_filepath) as json_data:
            try:
                project_data = json.load(json_data)
            except ValueError as e:
                print('Problem with the JSON parsing of project file. '+ str(e))                
                sys.exit()
                

        
            
        bag = rosbag.Bag(parsed.output_bag, 'w')
       
        visim_json_project = VISimProject()
        if (visim_json_project.fromJSON(project_data) == False):            
             print( 'Problem with the JSON file parsing. Look in the terminal')
             sys.exit()
        
        imu_filepath = os.path.join(parsed.project_folder, 'output/1_Rotors/imu_data.csv')
        gtpose_filepath = os.path.join(parsed.project_folder, 'output/1_Rotors/pose_data.csv')
    
        namespace = "/{0}".format(parsed.namespace) if parsed.namespace else ""

#add imu msg to the bag
        imu_topic = namespace + "/imu"
 #       imu_timestamps = list()
    
        with open(imu_filepath, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            headers = next(reader, None)
            print('loading imu data')
            for row in reader:
                imumsg, timestamp = createImuMessge(row[0], row[1:4], row[4:7])
                bag.write(imu_topic, imumsg, timestamp)
     #           imu_timestamps.append(timestamp)
                
#add gtpose msg to the bag
        gtpose_topic = namespace + "/ground_truth_imu_pose"
        pose_timestamps = list()
        
        with open(gtpose_filepath, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            headers = next(reader, None)
            print('loading gt pose data')
            for row in reader:
                posemsg, timestamp_pose = createGtPoseMessge(row[0], row[1:4], row[4:8])
                bag.write(gtpose_topic, posemsg, timestamp_pose)
                pose_timestamps.append(timestamp_pose)
            
#add image msg to the bag
        for cam_data in visim_json_project.cameras:
            cam_dirpath = os.path.join(parsed.project_folder, 'output/2_Blender/' + cam_data.cam_name + '_rgbd')
 #           cam_topic = namespace + "/{0}/image_raw".format(cam_data.cam_name)
            cam_info_topic = namespace + "/{0}/camera_info".format(cam_data.cam_name)
            cam_image_files = getImageFilesFromDir(cam_dirpath)
            
            print("loading camera: {0}".format(cam_data.cam_name))
            progress_total = len(cam_image_files)
            progress_last_percent = -1
            image_counter = 0
            for image_filename in cam_image_files:
                try:
                    idx = int(image_filename[3:-4])-1
                    timestamp = pose_timestamps[idx]
                    timestamp.nsecs +=1 #add one nano second to garanty the image will be sorted after the imu at the same timestamp
                    image_msgs = loadImagesToRosMsg(timestamp,cam_data,cam_dirpath,image_filename)
                    cam_info_msg = createCameraInfoMsg(timestamp,cam_data)
                    bag.write(cam_info_topic, cam_info_msg, timestamp)
                    for curr_img_msg in image_msgs:
                        curr_topic = namespace + "/{0}/{1}".format(cam_data.cam_name,curr_img_msg[0])
                        bag.write(curr_topic, curr_img_msg[1], timestamp)
                        
                   # bag.write(cam_topic, image_msg, timestamp)
                except IndexError as e:
                    print('image {0} ignored. Some expected in the end of the sequence'.format(idx))

                image_counter = image_counter+1;
                percent = math.floor(100*image_counter/progress_total)
                if percent != progress_last_percent:
                    print("progress: {0}/100".format(int(percent)), end='\r')
                    sys.stdout.flush()
                    progress_last_percent = percent
            print("")

    finally:
        bag.close()
    
