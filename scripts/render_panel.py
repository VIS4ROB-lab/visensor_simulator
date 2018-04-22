import bpy
import csv
import mathutils
import math
import os
import time
import json


# create a camera_setting in the type
# create a global property about selected camera


## Dataformat for the JSON project

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

def test_VISimProject():
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


################ Core

class VISimCameraSetting(bpy.types.PropertyGroup):
    has_config = bpy.props.BoolProperty(
        name="has visim configuration",
        description="Indicates if this camera,\n"
                    " has a VISim configuration",
        default=False
    )
    height = bpy.props.IntProperty(
        name="Image height",        
        default=0
    )
    width = bpy.props.IntProperty(
        name="Image width",        
        default=0
    )
    cam_name = bpy.props.StringProperty(
        name="cam_name",
        default="??"
    )
    project_folder = bpy.props.StringProperty(
        name="Project Folder",
        default="??",
        subtype = 'DIR_PATH'
    )


def create_camera( visim_camera, parent, project_folder,cam_list):
    #create camera instance
    
    camera_data_name =     parent.name + "_" + visim_camera.cam_name + "_data" 
    if camera_data_name in bpy.data.cameras:
        bpy.data.cameras.remove(bpy.data.cameras[camera_data_name])
        
    camera_data = bpy.data.cameras.new( camera_data_name )
    camera_object = bpy.data.objects.new( visim_camera.cam_name, camera_data )
    bpy.context.scene.objects.link( camera_object )
    camera_object.parent = parent

    #configure
    camera_data.angle = 2*math.atan2( visim_camera.width/2.0,visim_camera.focal_lenght )
    camera_data.visim_config.has_config = True
    camera_data.visim_config.height = visim_camera.height
    camera_data.visim_config.width = visim_camera.width
    camera_data.visim_config.cam_name = visim_camera.cam_name
    camera_data.visim_config.project_folder = project_folder
    cam_list[visim_camera.cam_name] = camera_object
    
def load_trajectory(self, context, filepath):
    return None

class RosPose:
    q = []
    p = []
    q_BlenderRos = mathutils.Quaternion([0,1,0,0]) # converting from ros to blender is a 180 rotation around x
    timestamp = ''
    
    def transformed(self, rhs ):
        result = RosPose()        
        result.q = self.q * rhs.q
        
        rotated_p_rhs = rhs.p.copy()
        rotated_p_rhs.rotate(self.q)
        result.p = self.p + rotated_p_rhs
        return result

    def __init__(self, position=mathutils.Vector((0,0,0)), orientation = mathutils.Quaternion([1,0,0,0]),timestamp=''):
        self.q = orientation
        self.p = position
        self.timestamp = timestamp
        
    def __str__(self):
        return str(self.p) + " " + str(self.q)

    

class BodyTrajectory:
    poses = []
    
    def __init__(self,csv_parsed_list):
        for p in csv_parsed_list:
            curr_pose = RosPose(mathutils.Vector((p[1],p[2],p[3])),mathutils.Quaternion([p[7],p[4],p[5],p[6]]),p[0])
            self.poses.append(curr_pose)
    
    

def process_project_file(self, context, filepath):

#open the json file
    with open(filepath) as json_data:
        project_data = json.load(json_data)

        visim_project = VISimProject()
        if (visim_project.fromJSON(project_data) == False):
            self.report({'ERROR'}, 'Problem with the JSON file parsing. Look in the terminal')
            print(json.dumps(project_data,sort_keys=False, indent=4))
            return {'CANCELLED'}
        
        

#set object mode- I dont know why but it is part of the tutorial
        if(bpy.context.scene.objects.active == None):
            #select any object
            bpy.context.scene.objects.active = bpy.context.scene.objects.values().pop()

        bpy.ops.object.mode_set(mode='OBJECT')


        project_name = visim_project.name

        if project_name in bpy.context.scene.objects:
            self.report({'ERROR'}, 'Project already exist- Please delete the hierarchy')
            return {'CANCELLED'}

        root_output_folder = os.path.dirname(os.path.abspath(filepath))
        
        poses_filename  = os.path.join(root_output_folder,'output/1_Rotors/pose_data.csv')
        print (poses_filename)
        
        try:
            
            with open(poses_filename, 'r') as poses_file_h:
                next(poses_file_h) #jump first line
                reader = csv.reader(poses_file_h)
                body_poses_list = list(reader)
        except EnvironmentError:
            print('error')
            self.report({'ERROR'}, 'pose file could not be opened: '+poses_filename)
            return {'CANCELLED'}
            
        if body_poses_list.count < 1:
            self.report({'ERROR'}, 'empty pose file ' + poses_filename)
            return {'CANCELLED'}
            
        body_trajectory = BodyTrajectory(body_poses_list)

        #create a empty to hold the cameras
        project_empty_obj = bpy.data.objects.new( project_name , None )

        #add to the current scene
        bpy.context.scene.objects.link( project_empty_obj )

        #draw empty
        project_empty_obj.empty_draw_size = 2
        project_empty_obj.empty_draw_type = 'PLAIN_AXES'
        
        cam_list = {}

        for curr_cam in visim_project.cameras:
            create_camera(curr_cam,project_empty_obj,root_output_folder,cam_list)
            
        
            
        for curr_cam_obj in cam_list:
            load_trajectory( curr_cam_obj, body_trajectory  )

    return {'FINISHED'}

################ GUI

class VISimRenderOperator(bpy.types.Operator):
    bl_idname = "visim.render"
    bl_label = "Render Camera"

    def execute(self, context):
        print("Hello World")
        #process_project_file(self,context,'/home/lucas/data/vi-sensor_lite_simulator/project_test/test_project.json')
            
        cam = bpy.data.objects['cam0']
        
        imu_quat = mathutils.Quaternion([1,0,0,0])
        imu_vec = mathutils.Vector((1,0,0))#*mathutils.Quaternion([0,1,0,0])
        b2r_quat = mathutils.Quaternion([0,1,0,0])   
        qc45x_quat = mathutils.Quaternion([0.924,0.383,0,0]) 
        downlooking_quat = mathutils.Quaternion([0, -0.7071067811865475, 0.7071067811865476, 0])
        
        cquat = mathutils.Quaternion([0.5,0.5,-0.5,0.5])
        cvec = mathutils.Vector((0,0,0))
        rpose = RosPose(imu_vec,imu_quat)
        cpose = RosPose(cvec,downlooking_quat*b2r_quat)
        repose = rpose.transformed(cpose)
        print(rpose)
        print(cpose)
        print(repose)
        cam.location = repose.p
        cam.rotation_mode = "QUATERNION"
        cam.rotation_quaternion = repose.q
        #cam.keyframe_insert('location')
#        mat_rot = quat.to_matrix().to_4x4()
#        mat_trans = mathutils.Matrix.Translation(vec)
#        mat = mat_trans * mat_rot
#        print(mat)
        
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return context.scene.visim_camera is not None




class VISimRenderPanel(bpy.types.Panel):
    bl_idname = "RENDER_PT_visim_render_panel"
    bl_label = "VISim Render"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"

    def draw(self, context):
        self.layout.prop(context.scene, "visim_camera", expand=True)
        self.layout.operator(VISimRenderOperator.bl_idname)
        



# Register
classes = ( VISimRenderOperator, VISimRenderPanel , VISimCameraSetting )


def scene_visim_camera_poll(self, object):
    return object.visim_config.has_config == True

def scene_visim_camera_update(self, context):
    return None
    

def register():    
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Camera.visim_config =  bpy.props.PointerProperty(type=VISimCameraSetting) 
    bpy.types.Scene.visim_camera = bpy.props.PointerProperty(type=bpy.types.Camera,update=scene_visim_camera_update,poll=scene_visim_camera_poll,name="VI Camera")


def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)

    del bpy.types.Scene.visim_camera
    del bpy.types.Camera.visim_config

if __name__ == "__main__":
    register()




# ###############################
# ## Portal planebpy.types.Camera
# ###############################

# def is_wowgroup_enabled(self, obj):
#     return True


# class WowPortalPlanePanel(bpy.types.Panel):
#     bl_idname = "OBJ_PT_wow_test_panel"
#     bl_space_type = "PROPERTIES"
#     bl_region_type = "WINDOW"
#     bl_context = "object"
#     bl_label = "Wow Portal Plane"
#     bl_options = {'DEFAULT_CLOSED'}

#     def draw_header(self, context):
#         layout = self.layout
#         self.layout.prop(context.object.WowPortalPlane, "Enabled")

#     def draw(self, context):
#         layout = self.layout
#         row = layout.row()
#         layout.enabled = context.object.WowPortalPlane.Enabled
#         self.layout.prop(context.object.WowPortalPlane, "First")
#         self.layout.prop(context.object.WowPortalPlane, "Second")     

#     @classmethod
#     def poll(cls, context):
#         return (context.object is not None and context.object.data is not None and isinstance(context.object.data,bpy.types.Mesh))


# class WowPortalPlanePropertyGroup(bpy.types.PropertyGroup):
#     Enabled = bpy.props.BoolProperty(name="", description="Enable wow WMO group properties")
#     First = bpy.props.BoolProperty(name="asd", description="Enable wow WMO group properties")
#     Second = bpy.props.BoolProperty(name="asdd", description="Enable wow WMO group properties")
#     PortalID = bpy.props.IntProperty(name="Portal's ID", description="Portal ID")


# def RegisterWowPortalPlaneProperties():
#     bpy.types.Object.WowPortalPlane = bpy.props.BoolProperty(name="assssd", description="Enable wow WMO group properties")


# def UnregisterWowPortalPlaneProperties():
#     bpy.types.Object.WowPortalPlane = None

# def register():
#     RegisterWowPortalPlaneProperties()
#     bpy.utils.register_class(WowPortalPlanePanel)

# if __name__ == "__main__":
#     register()


# import bpy


# class ObjectSelectPanel(bpy.types.Panel):
#     bl_idname = "OBJECT_PT_select"
#     bl_label = "Select"
#     bl_space_type = 'PROPERTIES'
#     bl_region_type = 'WINDOW'
#     bl_context = "object"
#     bl_options = {'DEFAULT_CLOSED'}

#     @classmethod
#     def poll(cls, context):
#         return (context.object is not None)

#     # def draw_header(self, context):
#     #     layout = self.layout
#     #     obj = context.object
#     #     layout.prop(obj, "select", text="")

#     def draw(self, context):
#         layout = self.layout

#         obj = context.object
#         row = layout.row()
#         row.prop(obj, "hide_select")
#         row.prop(obj, "hide_render")

#         box = layout.box()
#         box.label("Selection Tools")
#         box.operator("object.select_all").action = 'TOGGLE'
#         row = box.row()
#         row.operator("object.select_all").action = 'INVERT'
#         row.operator("object.select_random")


# bpy.utils.register_class(ObjectSelectPanel)