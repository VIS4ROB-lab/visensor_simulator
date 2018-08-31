#!/usr/bin/env python
# -*- coding: utf-8 -*-
#License (BSD)
#Copyright (c) 2018, Lucas Teixeira, Vision for Robotics Lab, ETH Zurich, Switzerland



from __future__ import print_function

import os
import argparse
import sys
import OpenEXR
import Imath
import glob


def convertExr(input_file,output_file):    
    in_image_exr = OpenEXR.InputFile(input_file)  
    header = in_image_exr.header()
    header['compression'] = Imath.Compression(Imath.Compression.NO_COMPRESSION)
    out_image_exr = OpenEXR.OutputFile(output_file,header)      
    channels = header['channels'].keys()
    newchannels = dict(zip(channels, in_image_exr.channels(channels)))
    out_image_exr.writePixels(newchannels)

    

if __name__ == "__main__":

    #setup the argument list
    parser = argparse.ArgumentParser(description='Tool to convert old render that used compression on the exr files. It saves about 30% on space but it is 50 times slower to open.')
    parser.add_argument('--input_folder',   nargs='?', help='input folder')
    parser.add_argument('--output_folder',   nargs='?', help='output folder')
    
    #print help if no argument is specified
    if len(sys.argv)<2:
        parser.print_help()
        sys.exit(0)

    #parse the args
    parsed = parser.parse_args()

    if not os.path.exists(parsed.input_folder) :
        print('The input folder does not exist :'+ parsed.input_folder)
        sys.exit()
        
    if os.path.exists(parsed.output_folder):#prevent a mess and to right on the input folder
        print('The output folder already exist. this script need to create a new output folder')
        sys.exit()
        
        
        
    original_current_folder = os.getcwd()
    
    os.chdir(parsed.input_folder)
    files_to_convert = glob.glob('*.exr')
    os.chdir(original_current_folder)
    
    if(len(files_to_convert) <= 0):
        print('No .exr files found in the input folder:' + parsed.input_folder)
        sys.exit()

    try:
        os.mkdir(parsed.output_folder)
    
        for f in files_to_convert:
            print('input file is:'+ os.path.join(parsed.input_folder,f))
            print('output file is:'+ os.path.join(parsed.output_folder,f)+'\n')
            convertExr(os.path.join(parsed.input_folder,f),os.path.join(parsed.output_folder,f))
            
    except ValueError:
        print("Oops!")

