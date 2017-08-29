import bpy
import csv
import mathutils
import math
import os
import time


bl_info = {
    "name": "Ros pose(CSV) format",
    "description": "Import-Export Ros Poses.",
    "author": "Lucas Teixeira",
    "version": (0, 1),
    "blender": (2, 74, 0),
    "location": "File > Import-Export",
    "warning": "", # used for warning icon and text in addons panel
    "wiki_url": "lteixeira@mavt.ethz.ch"
                "Scripts/My_Script",
    "category": "Import-Export"}

def read_ros_pose_file(self,context, filepath):

    #get the active object that should be the camera
    obj = bpy.context.active_object

    if obj.type != 'CAMERA':
        self.report({'ERROR'}, 'Please select one camera')
        return {'CANCELLED'}

    
    bpy.context.active_object.animation_data_clear()
    with open(filepath, 'r') as f:
        next(f) #jump first line
        reader = csv.reader(f)
        positions = list(reader)
    
    

    #create output folder
    timestr = time.strftime("blender_result_%Y-%m-%d-%H-%M-%S")
    root_output_folder = os.path.join(os.path.dirname(os.path.abspath(filepath)),timestr)
    images_output_folder = os.path.join(root_output_folder,'cam0/')
    os.makedirs(images_output_folder)

    id_time_filename  = os.path.join(root_output_folder,'blender_id_time.csv')


    image_step = 10 # vi-sensor camera runs at 20 Hz and the imu at 200 Hz
    keyframe_counter = 1
    t = dict(time=0,px=1,py=2,pz=3,qx=4,qy=5,qz=6,qw=7) #pose
    
    with open(id_time_filename, 'w') as id_time_file:
        for i in range(0,len(positions),image_step ):
            #print([positions[i][t['px']],positions[i][t['py']],positions[i][t['pz']]])
            keyframe_id = keyframe_counter;
            #image_time = positions[i][t['time']]
            #print(image_time)
            image_time = (positions[i][t['time']])[:-1]+'1' # uggly ... add one nanosec on the image timestamp to make sure that imu measurement is sorted before the image.
            id_time_file.write('{:08d},{}\n'.format(keyframe_id, image_time))
            bpy.context.scene.frame_set(keyframe_id)
            obj.location = [float(positions[i][t['px']]),float(positions[i][t['py']]),float(positions[i][t['pz']])]
            obj.keyframe_insert('location')
            obj.rotation_mode = "QUATERNION"            
            imu_orientation = mathutils.Quaternion([float(positions[i][t['qw']]),float(positions[i][t['qx']]),float(positions[i][t['qy']]),float(positions[i][t['qz']])])
            cam_imu_rotation = mathutils.Quaternion([0.0, 1.0, 0.0],math.radians(-90))
            obj.rotation_quaternion = imu_orientation * cam_imu_rotation * mathutils.Quaternion([0.0, 0.0, 1.0],math.radians(-90))
            obj.keyframe_insert('rotation_quaternion')
            keyframe_counter = keyframe_counter+1;


    #setup scene
    scene = bpy.context.scene
    scene.render.filepath = os.path.join(images_output_folder,'rgb_########.png')# 8 zeros padding
    scene.render.resolution_x = 752
    scene.render.resolution_y = 480
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGB'
    scene.render.image_settings.color_depth = '8'
    scene.render.image_settings.compression = 0

    # scene.render.image_settings.file_format = 'OPEN_EXR'
    # scene.render.image_settings.exr_codec = 'PIZ'
    # scene.render.image_settings.color_depth = '32'
    # scene.render.image_settings.color_mode = 'RGB'
    # scene.render.image_settings.use_zbuffer = True

    #setup the camera    
    focal_length = 455
    obj.data.angle = 2*math.atan2(scene.render.resolution_x/2.0,focal_length)

    return {'FINISHED'}


# ImportHelper is a helper class, defines filename and
# invoke() function which calls the file selector.
from bpy_extras.io_utils import ImportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator


class ImportRosPoses(Operator, ImportHelper):
    """This appears in the tooltip of the operator and in the generated docs"""
    bl_idname = "import_ros_poses.read_data"  
    bl_label = "Import Ros Poses"

    # ImportHelper mixin class uses this
    filename_ext = ".csv"

    filter_glob = StringProperty(
            default="*.csv",
            options={'HIDDEN'},
            maxlen=255,  # Max internal buffer length, longer would be clamped.
            )
 
    def execute(self, context):
        return read_ros_pose_file(self,context, self.filepath)


# Only needed if you want to add into a dynamic menu
def menu_func_import(self, context):
    self.layout.operator(ImportRosPoses.bl_idname, text="ROS Pose File(*.csv)")


def register():
    bpy.utils.register_class(ImportRosPoses)
    bpy.types.INFO_MT_file_import.append(menu_func_import)


def unregister():
    bpy.utils.unregister_class(ImportRosPoses)
    bpy.types.INFO_MT_file_import.remove(menu_func_import)


if __name__ == "__main__":
    register()

    # test call
    #bpy.ops.import_ros_poses.read_data('INVOKE_DEFAULT')
