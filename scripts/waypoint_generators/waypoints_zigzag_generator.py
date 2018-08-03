import argparse
import sys
import math



def main():    
    
    #setup the argument list
    parser = argparse.ArgumentParser(description='Generate a zig-zag trajectory (ex. -o testfile.txt  ')
    parser.add_argument('-o','--output',  metavar='filename', help='output waypoint file')
    parser.add_argument('-bbn','--bound_box_north',  metavar='value', type=float,default=80,  help='max lenght of the trajectory in the north direction (in meters)')
    parser.add_argument('-bbe','--bound_box_east',  metavar='value', type=float,default=80,  help='max lenght of the trajectory in the east direction (in meters)')
    parser.add_argument('-bbu','--bound_box_up',  metavar='value', type=float,default=0, help='max lenght of the trajectory in the up direction (in meters)')
    
    parser.add_argument('-sn','--step_north',  metavar='value', type=float,default=10,  help='step size in the north direction (in meters)')
    parser.add_argument('-se','--step_east',  metavar='value', type=float,default=80,  help='step size in the east direction (in meters)')
    parser.add_argument('-su','--step_up',  metavar='value', type=float,default=0, help='step size in the up direction (in meters)')
    
    parser.add_argument('-on','--offset_north',  metavar='value', type=float,default=-40, help='offset on north direction (in meters)')
    parser.add_argument('-oe','--offset_east',  metavar='value', type=float,default=40, help='offset on east direction (in meters)')
    parser.add_argument('-ou','--offset_up',  metavar='value', type=float,default=20, help='offset on up direction (in meters). Simulators usually cannot work with up= 0')
    parser.add_argument('-he','--heading',  metavar='value', type=float,default=0, help='desired constant heading-yaw (degrees)')
    parser.add_argument('-gp','--gimbal_pitch',  metavar='value', type=float,default=90, help='desired constant gimbal pitch (degrees)')
    

#todo Add validation to the input    
    #print help if no argument is specified
    if len(sys.argv)<2:
        parser.print_help()
        sys.exit(0)
    
    #parse the args
    parsed = parser.parse_args()
    
    if parsed.bound_box_east == 0 or (parsed.bound_box_north == 0 and parsed.bound_box_up == 0)  :
        print "error"
        sys.exit(0)
        
    if parsed.step_east <= 0:
        parsed.step_east = parsed.bound_box_east
    
    
    
    num_rows_east = int(math.floor(parsed.bound_box_east/parsed.step_east) + 1)
    
    
    num_rows_up = 1 if parsed.step_up <= 0 else (parsed.bound_box_up/parsed.step_up)+1
    num_rows_north = 1 if parsed.step_north <= 0 else (parsed.bound_box_north/parsed.step_north)+1
    
    num_rows = int(math.floor(max(num_rows_up,num_rows_north)))
    
    print "the output has {} rows and {} cols".format(num_rows,num_rows_east)
    
    going_east = True
    
    with open(parsed.output,'w') as output_file:
        for row in range(num_rows):
            n = min(row*parsed.step_north,parsed.bound_box_north) +parsed.offset_north
            u = min(row*parsed.step_up,parsed.bound_box_up)+parsed.offset_up
            for col in range(num_rows_east):
                e =  min(col*parsed.step_east,parsed.bound_box_east)
                e_directional = e if going_east else parsed.bound_box_east-e
                output_file.write('{0:.2f},{1:.2f},{2:.2f},{3:.2f},{4:.2f}\n'.format( n,-e_directional+parsed.offset_east,u,parsed.heading,parsed.gimbal_pitch))
            going_east = not going_east
    
    print "done!"            

if __name__ == "__main__":
    main()