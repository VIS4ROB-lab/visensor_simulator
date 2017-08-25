import numpy as np
import argparse
import sys
import math

#TODO find a better name for vertical and horizontal and the correlation with x,y,z

def calculate_height(fovh,img_h,gsd):
    height = gsd*img_h/(2*math.tan(fovh/2))
    return height
    
def calculate_spacing(gsd,resolution,overlap):
    space = gsd*resolution*overlap*0.01
    return space

def main():    
    
    #setup the argument list
    parser = argparse.ArgumentParser(description='Generate a zig-zag trajectory (ex. -o testfile.txt  -resh 752 -resv 480 -glv 50 -glh 20 ')
    parser.add_argument('-o','--output',  metavar='filename', help='output waypoint file')
    parser.add_argument('-glh','--ground_lenght_horizontal',  metavar='value', type=float,default=100,  help='ground lenght horizontal (in meters)')
    parser.add_argument('-glv','--ground_lenght_vertical',  metavar='value', type=float,default=200,  help='ground lenght vertical (in meters)')
    parser.add_argument('-gsd','--ground_sample_distance',  metavar='value', type=float,default=0.01, help='ground sample distance (in meters)')
    parser.add_argument('-fovh','--field_of_view_horizontal',  metavar='value', type=float,default=65, help='field of view - horizontal(in degrees)')
    parser.add_argument('-resh','--image_resolution_horizontal',  metavar='value', type=float,default=2560, help='image resolution - horizontal( in pixels)')
    parser.add_argument('-resv','--image_resolution_vertical',  metavar='value', type=float,default=1920, help='image resolution - vertical( in pixels)')
    parser.add_argument('-ovh','--overlap_horizontal',  metavar='value', type=float,default=80, help='overlap between images - vertical( in percentage[0-100])')
    parser.add_argument('-ovv','--overlap_vertical',  metavar='value', type=float,default=80, help='overlap between images - vertical( in percentage [0-100])')

#todo Add validation to the input    
    #print help if no argument is specified
    if len(sys.argv)<2:
        parser.print_help()
        sys.exit(0)
    
    #parse the args
    parsed = parser.parse_args()
    
    height = calculate_height(parsed.field_of_view_horizontal,parsed.image_resolution_vertical,parsed.ground_sample_distance)
    sample_distance_horizontal = calculate_spacing(parsed.ground_sample_distance,parsed.image_resolution_horizontal,parsed.overlap_horizontal)
    sample_distance_vertical = calculate_spacing(parsed.ground_sample_distance,parsed.image_resolution_vertical,parsed.overlap_vertical)
    
    vertical_positions = np.linspace(0,parsed.ground_lenght_vertical,parsed.ground_lenght_vertical/sample_distance_vertical, endpoint=True)
    horizontal_positions = np.linspace(0,parsed.ground_lenght_horizontal,parsed.ground_lenght_horizontal/sample_distance_horizontal, endpoint=True)
    
    going_right = True
    
    with open(parsed.output,'w') as output_file:
        for u in vertical_positions:        
            for v in horizontal_positions:
                v_directional = v if going_right else parsed.ground_lenght_horizontal-v
                output_file.write('{0:.2f} {1:.2f} {2:.2f} {3:.2f}\n'.format( u,v_directional,height,0))
            going_right = not going_right
    
    print "done!"            

if __name__ == "__main__":
    main()