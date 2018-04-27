import bpy
import json
import csv
import mathutils
import math
import os
import time
import bpy_extras


bl_info = {
    "name": "VISim project format",
    "description": "Import and Render a VIsim project",
    "author": "Lucas Teixeira",
    "version": (0, 1),
    "blender": (2, 79, 0),
    "location": "File > Import-Export",
    "warning": "", # used for warning icon and text in addons panel
    "wiki_url": "lteixeira@mavt.ethz.ch"
                "Scripts/My_Script",
    "category": "Import-Export"}
    

## Dataformat for the JSON project

class VISimCamera():
    cam_name = "cam_default"
    focal_lenght = 455
    frequency_multiplier = 10 # if 10 then it is 10 times SLOWER than the imu that runs at 200Hz
    height = 480
    width = 752
    transform_ImuCamera = [0, -1, 0, 0, #frontal camera in ros standard
                          -1, 0, 0, 0,
                          0, 0, -1, 0,
                          0, 0, 0, 1] 
    
    
    
    def toJSON(self):
        result = {}
        result["cam_name"] = self.cam_name
        result["focal_lenght"] = self.focal_lenght
        result["frequency_multiplier"] = self.frequency_multiplier
        result["height"] = self.height
        result["width"] = self.width
        result["T_SC"] = self.transform_ImuCamera
        return result
    
    def fromJSON(self, json_dict):
        try:
            self.cam_name = json_dict["cam_name"]
            self.focal_lenght = json_dict["focal_lenght"]
            self.frequency_multiplier = json_dict["frequency_multiplier"]
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

def test_VISimProject():
    proj = VISimProject()
    proj.name = "project_test"
    cam0 = VISimCamera()
    cam0.cam_name = "cam0"
    
    
    cam1 = VISimCamera()    
    cam1.cam_name = "cam1"
    
    
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

################### core


class VISimProjectLoader():

    @staticmethod
    def create_camera( visim_camera, parent,cam_list):
    #create camera instance
    
        camera_obj_name =     parent.name[6:] + "_" + visim_camera.cam_name 
        if camera_obj_name in bpy.data.cameras:
            bpy.data.cameras.remove(bpy.data.cameras[camera_obj_name])
            
        camera_data = bpy.data.cameras.new( camera_obj_name )
        camera_object = bpy.data.objects.new( camera_obj_name, camera_data )
        bpy.context.scene.objects.link( camera_object )
        camera_object.parent = parent
    
        #configure
        camera_data.angle = 2*math.atan2( visim_camera.width/2.0,visim_camera.focal_lenght )
        camera_data.visim_cam_config.has_config = True
        camera_data.visim_cam_config.height = visim_camera.height
        camera_data.visim_cam_config.width = visim_camera.width
        camera_data.visim_cam_config.cam_name = visim_camera.cam_name
        L = visim_camera.transform_ImuCamera
        tf = mathutils.Matrix([L[0:4],L[4:8],L[8:12],L[12:16]])
        pose = tf.decompose()
        camera_data.visim_cam_config.imu_camera_translation = pose[0]
        camera_data.visim_cam_config.imu_camera_quaternion  = pose[1]
        cam_list[visim_camera.cam_name] = camera_object
        
    @staticmethod
    def load_project(operator, project_object , filepath = None):
        
        if project_object != None:
            filepath = os.path.join(project_object.visim_project_setting.project_folder, 'visim_project.json')
            
            #open the json file
        with open(filepath) as json_data:
            try:
                project_data = json.load(json_data)
            except json.decoder.JSONDecodeError as e:
                operator.report({'ERROR'}, 'Problem with the JSON file parsing. '+ str(e))                
                return {'CANCELLED'}
    
            visim_json_project = VISimProject()
            if (visim_json_project.fromJSON(project_data) == False):
                operator.report({'ERROR'}, 'Problem with the JSON file parsing. Look in the terminal')
                #print(json.dumps(project_data,sort_keys=False, indent=4))
                return {'CANCELLED'}
                
        #set object mode- I dont know why but it is part of the tutorial
        if(bpy.context.scene.objects.active == None):
            #select any object
            bpy.context.scene.objects.active = bpy.context.scene.objects.values().pop()

        bpy.ops.object.mode_set(mode='OBJECT')

        if project_object == None:

            project_name = 'visim_' + visim_json_project.name

            if project_name in bpy.context.scene.objects:
                operator.report({'ERROR'}, 'Project with the same name already exist- Please delete the hierarchy ')
                return {'CANCELLED'}

            #create a empty to hold the cameras
            project_object = bpy.data.objects.new( project_name , None )
            project_object.empty_draw_size = 2
            project_object.empty_draw_type = 'PLAIN_AXES'
            #'IPO_BACK'

            #add to the current scene
            bpy.context.scene.objects.link( project_object )

            root_output_folder = os.path.dirname(os.path.abspath(filepath))
            project_object.visim_project_setting.has_config = True
            project_object.visim_project_setting.project_folder = root_output_folder

        else:
            #delete all children
            for child in project_object.children:
                if child.type == 'CAMERA':
                    bpy.data.cameras.remove(child.data)
                else:
                    operator.report({'ERROR'}, 'Unexpected type :'+ str(child.data))
                    return {'CANCELLED'}
                #bpy.context.scene.objects.unlink(child)
                #bpy.data.objects.remove(child)



        cam_list = {}

        for curr_cam in visim_json_project.cameras:
            VISimProjectLoader.create_camera(curr_cam,project_object,cam_list)

        return {'FINISHED'}

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
    
    frequency_multiplier = bpy.props.IntProperty(
        name="Frequency Multiplier",
        default=10
    )
    
    cam_name = bpy.props.StringProperty(
        name="cam_name",
        default="??"
    )
    
    imu_camera_translation = bpy.props.FloatVectorProperty(
        name="imu_camera_translation",
        default=[0,0,0],
        subtype= 'TRANSLATION',
        precision=2
    )
    
    imu_camera_quaternion = bpy.props.FloatVectorProperty(
        name="imu_camera_quaternion",
        default=[1,0,0,0],
        subtype= 'QUATERNION',
        precision=6,
        size=4
    )
    
    
    
class VISimProjectObjectSetting(bpy.types.PropertyGroup):
    has_config = bpy.props.BoolProperty(
        name="has visim configuration",
        description="Indicates if this camera,\n"
                    " has a VISim configuration",
        default=False
    )
        
    project_folder = bpy.props.StringProperty(
        name="Project Folder",
        default="??",
        subtype = 'DIR_PATH'
    )

class VISimProjectReloadOperator(bpy.types.Operator):
    bl_idname = "visim.reload_project"
    bl_label = "Reload VISim Project"

    def execute(self, context):
        return VISimProjectLoader.load_project(self,context.object)
        

    @classmethod
    def poll(cls, context):
        return context.object.visim_project_setting.has_config == True

class VISimProjectPanel(bpy.types.Panel):
    bl_idname = "OBJ_PT_visim_project_object_panel"
    bl_label = "VISim Project"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "object"

    def draw(self, context):
        #self.layout.prop(context.scene, "visim_camera", expand=True)
        self.layout.operator(VISimProjectReloadOperator.bl_idname)   

class VISimRenderOperator(bpy.types.Operator):
    bl_idname = "visim.render"
    bl_label = "Render Camera"

    def execute(self, context):
        print("Hello World")
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return True #context.scene.visim_camera is not None




class VISimRenderPanel(bpy.types.Panel):
    bl_idname = "RENDER_PT_visim_render_panel"
    bl_label = "VISim Render"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"

    def draw(self, context):
        #self.layout.prop(context.scene, "visim_camera", expand=True)
        self.layout.operator(VISimRenderOperator.bl_idname)   

class ImportVISimProj(bpy.types.Operator, bpy_extras.io_utils.ImportHelper):
    """This appears in the tooltip of the operator and in the generated docs"""
    bl_idname = "import_vi_sim_proj.read_data"  
    bl_label = "Import VISim Project"

    # ImportHelper mixin class uses this
    filename_ext = ".json"

    filter_glob = bpy.props.StringProperty(
            default="*.json",
            options={'HIDDEN'},
            maxlen=255,  # Max internal buffer length, longer would be clamped.
            )
 
    def execute(self, context):
        return VISimProjectLoader.load_project(self,None,self.filepath)
        #return {'FINISHED'} #read_ros_pose_file(self,context, self.filepath)



def menu_func_import(self, context):
    self.layout.operator(ImportVISimProj.bl_idname, text="VISim Project (.json)")

classes = (
    ImportVISimProj,
    VISimProjectPanel,
    VISimProjectReloadOperator,
    VISimRenderPanel,
    VISimRenderOperator,
    VISimCameraSetting,
    VISimProjectObjectSetting,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.INFO_MT_file_import.append(menu_func_import)

    bpy.types.Object.visim_project_setting =  bpy.props.PointerProperty(type=VISimProjectObjectSetting)
    bpy.types.Camera.visim_cam_config =  bpy.props.PointerProperty(type=VISimCameraSetting)


def unregister():
    bpy.types.INFO_MT_file_import.remove(menu_func_import)
    

    for cls in classes:
        bpy.utils.unregister_class(cls)
        
    del bpy.types.Object.visim_project_setting
    del bpy.types.Camera.visim_cam_config


if __name__ == "__main__":
    register()