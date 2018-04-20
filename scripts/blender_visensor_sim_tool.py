import bpy
import csv
import mathutils
import math
import os
import time
import json


bl_info = {
    "name": "VISimulator project format(*.json)",
    "description": "Import VISimulator project.",
    "author": "Lucas Teixeira",
    "version": (0, 2),
    "blender": (2, 79, 0),
    "location": "File > Import-Export",
    "warning": "", # used for warning icon and text in addons panel
    "wiki_url": "lteixeira@mavt.ethz.ch"
                "Scripts/My_Script",
    "category": "Import-Export"}



class VISimCamera():
    cam_name = "cam_default"
    focal_lenght = 455
    frequency_multiplier = 10 # if 10 then it is 10 times SLOWER than the imu that runs at 200Hz
    height = 480
    width = 752
    origin_pos = [0,0,0]
    origin_rpy = [0,0,0]
    
    
    def toJSON(self):
        result = {}
        result["cam_name"] = self.cam_name
        result["focal_lenght"] = self.focal_lenght
        result["frequency_multiplier"] = self.frequency_multiplier
        result["height"] = self.height
        result["width"] = self.width
        result["origin_pos"] = self.origin_pos
        result["origin_rpy"] = self.origin_rpy
        return result
    
    def fromJSON(self, json_dict):
        try:
            self.cam_name = json_dict["cam_name"]
            self.focal_lenght = json_dict["focal_lenght"]
            self.frequency_multiplier = json_dict["frequency_multiplier"]
            self.height = json_dict["height"]
            self.width = json_dict["width"]
            self.origin_pos = json_dict["origin_pos"]
            self.origin_rpy = json_dict["origin_rpy"]
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

def test_project():
    proj = VISimProject()
    proj.name = "project_test"
    cam0 = VISimCamera()
    cam0.cam_name = "cam0"
    cam0.origin_pos = [0.015,0.055,0.0065]
    
    cam1 = VISimCamera()    
    cam1.cam_name = "cam1"
    cam1.origin_pos = [0.015,-0.055,0.0065]
    
    proj.cameras.append(cam0)
    proj.cameras.append(cam1)
    
    proj_json = proj.toJSON()
    
    print("write")
    print(json.dumps(proj_json, 
                sort_keys=False, indent=4))
    proj_read = VISimProject()
    proj_read.fromJSON(proj_json)
    
    print("read")
    print(json.dumps(proj_read.toJSON(), 
                sort_keys=False, indent=4))

# def create_objs:
#     instance = bpy.data.objects.new('dupli_group', None)
#     instance.dupli_type = 'GROUP'
#     bpy.ops.object.mode_set(mode='OBJECT')
  
#     bpy.ops.object.camera_add(view_align=False, enter_editmode=False, location=(0,0,0), rotation=(0,0,0))
#     cam = bpy.context.active_object

#     #this will name the Camera Object
#     if 'Dolly_Camera' not in bpy.context.scene.objects:
#         cam.name = "Dolly_Camera"
#     else:
#         cam.name = "Dolly_Camera.000"

#     bpy.ops.object.mode_set(mode='OBJECT')

#     project_name = "lalalal"

#     #this will name the project Object
#     if project_name in bpy.context.scene.objects:
#         project_name += ".000"

#     o = bpy.data.objects.new( project_name , None )
#     bpy.context.scene.objects.link( o )

def create_camera( visim_camera, parent):
    camera_data = bpy.data.cameras.new(visim_camera.cam_name + "_data")
    camera_object = bpy.data.objects.new(visim_camera.cam_name,camera_data)
    bpy.context.scene.objects.link(camera_object)
    camera_object.parent = parent
    camera_data.angle = 2*math.atan2( visim_camera.width/2.0,visim_camera.focal_lenght)

def process_project_file(self, context, filepath):

    with open(filepath) as json_data:
        project_data = json.load(json_data)

        visim_project = VISimProject()
        if (visim_project.fromJSON(project_data) == False):
            self.report({'ERROR'}, 'Problem with the JSON file parsing. Look in the terminal')
            print(json.dumps(project_data,sort_keys=False, indent=4))
            return {'CANCELLED'}

        if(bpy.context.scene.objects.active == None):
            #select any object
            bpy.context.scene.objects.active = bpy.context.scene.objects.values().pop()

        bpy.ops.object.mode_set(mode='OBJECT')

        project_name = visim_project.name + "_imu"

        #this will name the project Object
        if project_name in bpy.context.scene.objects:
            project_name += ".000"

        project_empty_obj = bpy.data.objects.new( project_name , None )
        bpy.context.scene.objects.link( project_empty_obj )
        project_empty_obj.empty_draw_size = 2
        project_empty_obj.empty_draw_type = 'PLAIN_AXES'

        for curr_cam in visim_project.cameras:
            create_camera(curr_cam,project_empty_obj)

        


        


    return {'FINISHED'}

    #ROS camera coordinate system
    # +x should point to the right in the image
    # +y should point down in the image
    # +z should point into the plane of the image

    #Blender camera coordinate system
    # +x should point to the right in the image
    # -y should point down in the image
    # -z should point into the plane of the image

    #Conclusion
    # conversion between Ros and Blender camera system is a rotation of 180 around x
    


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


class ImportVISimProject(Operator, ImportHelper):
    """This appears in the tooltip of the operator and in the generated docs"""
    bl_idname = "import_visim_project.read_data"  
    bl_label = "Import VISimulator project"

    # ImportHelper mixin class uses this
    filename_ext = ".json"

    filter_glob = StringProperty(
            default="*.json",
            options={'HIDDEN'},
            maxlen=255,  # Max internal buffer length, longer would be clamped.
            )
 
    def execute(self, context):
        return process_project_file(self,context, self.filepath)


# Only needed if you want to add into a dynamic menu
def menu_func_import(self, context):
    self.layout.operator(ImportVISimProject.bl_idname, text="VISimulator project File(*.json)")


def register():
    bpy.utils.register_class(ImportVISimProject)
    bpy.types.INFO_MT_file_import.append(menu_func_import)


def unregister():
    bpy.utils.unregister_class(ImportVISimProject)
    bpy.types.INFO_MT_file_import.remove(menu_func_import)


if __name__ == "__main__":
    register()

#'/home/lucas/data/vi-sensor_lite_simulator/project_test/test_project.json'
    # test call
    #bpy.ops.import_ros_poses.read_data('INVOKE_DEFAULT')
