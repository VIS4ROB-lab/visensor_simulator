import bpy
import json
import csv
import mathutils
import math
import os
import time
import bpy_extras
import sys
import shutil


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
        result["focal_length"] = self.focal_length
        result["frequency_reduction_factor"] = self.frequency_reduction_factor
        result["height"] = self.height
        result["width"] = self.width
        result["T_SC"] = self.transform_ImuCamera
        return result
    
    def fromJSON(self, json_dict):
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
            curr_pose = RosPose(mathutils.Vector((float(p[1]),float(p[2]),float(p[3]))),mathutils.Quaternion([float(p[7]),float(p[4]),float(p[5]),float(p[6])]),p[0])
            self.poses.append(curr_pose)
    



class VISimProjectLoader():
    
    @staticmethod
    def prepare_render(operator, context):
        if context.scene.visim_render_camera is None:
            scene = context.scene
            scene.render.filepath = bpy.context.user_preferences.filepaths.temporary_directory# point the another place
            return {'FINISHED'} 
        
        project_object = context.scene.visim_render_camera.parent
        camera_data = context.scene.visim_render_camera.data
        
        #change scene camera
        context.scene.camera = context.scene.visim_render_camera
        
        project_object.hide = True        
        for child in project_object.children:
            if child.type == 'CAMERA':
                child.hide = True
            else:
                operator.report({'ERROR'}, 'Unexpected project child type :'+ str(child.data))
                return {'CANCELLED'}
                
        context.scene.camera.hide = False

        images_output_folder = os.path.join(project_object.visim_project_setting.project_folder,'output/2_Blender/'+camera_data.visim_cam_config.cam_name+'_rgbd')
        #if os.access(images_output_folder, os.R_OK | os.W_OK) :
        #    shutil.rmtree(images_output_folder)
        
        os.makedirs(images_output_folder,exist_ok=True)

        scene = context.scene
        scene.render.filepath = os.path.join(images_output_folder,'bl_############.png')# 10 zeros padding
        scene.render.resolution_x = camera_data.visim_cam_config.width
        scene.render.resolution_y = camera_data.visim_cam_config.height
        bpy.context.scene.frame_step = camera_data.visim_cam_config.frequency_reduction_factor
        
        if context.scene.output_image_format == 'PNG':
            scene.render.resolution_percentage = 100
            scene.render.image_settings.file_format = 'PNG'
            scene.render.image_settings.color_mode = 'RGB'
            scene.render.image_settings.color_depth = '8'
            scene.render.image_settings.compression = 0
        else:        
            scene.render.resolution_percentage = 100
            scene.render.image_settings.file_format = 'OPEN_EXR'
            scene.render.image_settings.exr_codec = 'PIZ'
            scene.render.image_settings.color_depth = '32'
            scene.render.image_settings.color_mode = 'RGB'
            scene.render.image_settings.use_zbuffer = True
        
        return {'FINISHED'}
        

        
    
    @staticmethod
    def load_trajectory(curr_cam_obj, body_trajectory):
        ros2blender_quat = mathutils.Quaternion([0,1,0,0])
        ctrans = curr_cam_obj.data.visim_cam_config.imu_camera_translation
        cquat  =  curr_cam_obj.data.visim_cam_config.imu_camera_quaternion
        T_BC = RosPose(ctrans,cquat*ros2blender_quat)
        keyframe_counter = 1
        nposes = len(body_trajectory.poses)
        last_percent = -1
        reduction_factor = curr_cam_obj.data.visim_cam_config.frequency_reduction_factor
        
        for T_WB in body_trajectory.poses:
            if (keyframe_counter % reduction_factor == 1):
                T_WC = T_WB.transformed(T_BC)
                bpy.context.scene.frame_set(keyframe_counter)
                curr_cam_obj.location = T_WC.p
                curr_cam_obj.keyframe_insert('location')
                curr_cam_obj.rotation_mode = "QUATERNION"
                # the rotation_quaternion is the rotation from camera to the parent
                curr_cam_obj.rotation_quaternion = T_WC.q
                curr_cam_obj.keyframe_insert('rotation_quaternion')
            keyframe_counter = keyframe_counter+1;
            percent = math.floor(100*keyframe_counter/nposes)
            if percent != last_percent:
                print("Trajectory loading progress: {}".format(percent), end='\r', flush=True)
                last_percent = percent
        print("")
        return None


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
        camera_object.hide_render = True
        camera_object.hide = True
    
        #configure
        camera_data.angle = 2*math.atan2( visim_camera.width/2.0,visim_camera.focal_length )
        camera_data.visim_cam_config.has_config = True
        camera_data.visim_cam_config.height = visim_camera.height
        camera_data.visim_cam_config.width = visim_camera.width
        camera_data.visim_cam_config.frequency_reduction_factor = visim_camera.frequency_reduction_factor        
        camera_data.visim_cam_config.cam_name = visim_camera.cam_name
        L = visim_camera.transform_ImuCamera
        tf = mathutils.Matrix([L[0:4],L[4:8],L[8:12],L[12:16]])
        pose = tf.decompose()
        camera_data.visim_cam_config.imu_camera_translation = pose[0]
        camera_data.visim_cam_config.imu_camera_quaternion  = pose[1]
        cam_list[visim_camera.cam_name] = camera_object
        

    @staticmethod
    def load_trajectories(operator, project_object):
        
        poses_filename  = os.path.join(project_object.visim_project_setting.project_folder,'output/1_Rotors/pose_data.csv')
        print (poses_filename)
        
        body_poses_list = []
        
        try:
            
            with open(poses_filename, 'r') as poses_file_h:
                next(poses_file_h) #jump first line
                reader = csv.reader(poses_file_h)
                body_poses_list = list(reader)
        except EnvironmentError:
            print('error')
            operator.report({'ERROR'}, 'pose file could not be opened: '+poses_filename)
            return {'CANCELLED'}
            
        if not body_poses_list:
            operator.report({'ERROR'}, 'empty pose file ' + poses_filename)
            return {'CANCELLED'}
            
        body_trajectory = BodyTrajectory(body_poses_list)
        
        bpy.context.scene.frame_start = 1        
        bpy.context.scene.frame_end = len(body_trajectory.poses)
        bpy.context.scene.frame_step = 1        
        
        ncamera= len(project_object.children)
        curr_cam = 0
        for child in project_object.children:
            curr_cam +=1
            print("current camera:{}/{}".format(curr_cam,ncamera))
            if child.type == 'CAMERA':
                VISimProjectLoader.load_trajectory(child,body_trajectory)
            else:
                operator.report({'ERROR'}, 'Unexpected project child type :'+ str(child.data))
                return {'CANCELLED'}
                    
        return {'FINISHED'}
    
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
            project_object.hide_render = True
            project_object.hide = True
            #'IPO_BACK'

            #add to the current scene
            bpy.context.scene.objects.link( project_object )

            root_output_folder = os.path.dirname(os.path.abspath(filepath))
            project_object.visim_project_setting.has_config = True
            project_object.visim_project_setting.project_folder = root_output_folder
            bpy.context.scene.objects.active = project_object

        else:
            #delete all children
            for child in project_object.children:
                if child.type == 'CAMERA':
                    bpy.data.cameras.remove(child.data)
                else:
                    operator.report({'ERROR'}, 'Unexpected project child type :'+ str(child.data))
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
    
    frequency_reduction_factor = bpy.props.IntProperty(
        name="Frequency Reduction Factor",
        default=88
    )
    
    cam_name = bpy.props.StringProperty(
        name="cam_name",
        default="??"
    )
    
    imu_camera_translation = bpy.props.FloatVectorProperty(
        name="imu_camera_translation",
        default=[0,0,0],
        subtype= 'TRANSLATION',
        precision=3
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

class VISimTrajectoryReloadOperator(bpy.types.Operator):
    bl_idname = "visim.reload_trajectory"
    bl_label = "Reload VISim only trajectory"

    def execute(self, context):
        return VISimProjectLoader.load_trajectories(self,context.object)
        

    @classmethod
    def poll(cls, context):
        return context.object.visim_project_setting.has_config == True

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
        self.layout.operator(VISimTrajectoryReloadOperator.bl_idname)
        self.layout.operator(VISimProjectReloadOperator.bl_idname)
        


class VISimRaytraceRenderOperator(bpy.types.Operator):
    bl_idname = "visim.blender_render"
    bl_label = "Raytrace Render"

    def execute(self, context):
        
        bpy.ops.render.render('INVOKE_DEFAULT',animation=True)      
        
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return context.scene.visim_render_camera is not None
        
class VISimOGLRenderOperator(bpy.types.Operator):
    bl_idname = "visim.opengl_render"
    bl_label = "OGL Render"

    def execute(self, context):
        
        # Call user prefs window
        bpy.ops.screen.userpref_show('INVOKE_DEFAULT')
        # Change area type
        area = bpy.context.window_manager.windows[-1].screen.areas[0]
        area.type = 'VIEW_3D'
        area.spaces[0].region_3d.view_perspective = 'CAMERA'
        area.spaces[0].viewport_shade = 'MATERIAL'
        
        
        output_folder = os.path.dirname(os.path.abspath(bpy.context.scene.render.filepath))
        if os.path.exists(output_folder) and os.path.isdir(output_folder):
            if os.listdir(output_folder):            
                bpy.context.window_manager.popup_menu(scene_visim_info_replace_files_draw, title="Alert", icon='RADIO')
        else:
            self.report({'ERROR'},"Given Output Directory don't exists")
            return {'FINISHED'}
        
        
        bpy.ops.render.opengl('INVOKE_DEFAULT',animation=True)
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return ((context.scene.visim_render_camera is not None) and context.scene.output_image_format == 'PNG') 


class VISimPrepareRenderOperator(bpy.types.Operator):
    bl_idname = "visim.prerender"
    bl_label = "Reload camera"

    def execute(self, context):
                       
        return VISimProjectLoader.prepare_render(self,context)

    @classmethod
    def poll(cls, context):
        return context.scene.visim_render_camera is not None


class VISimRenderPanel(bpy.types.Panel):
    bl_idname = "RENDER_PT_visim_render_panel"
    bl_label = "VISim Render"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"

    def draw(self, context):
        layout = self.layout        
        layout.prop(context.scene, "visim_render_camera", expand=True)
        layout.operator(VISimPrepareRenderOperator.bl_idname)   
         
        layout.label("Image format:")
        layout.prop(context.scene, "output_image_format", expand=True)
        
        row = layout.row(align=True)
        row.alignment = 'EXPAND'
        row.operator(VISimOGLRenderOperator.bl_idname, icon='RENDER_ANIMATION')
        row.operator(VISimRaytraceRenderOperator.bl_idname, icon='RENDER_ANIMATION')

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
        val = VISimProjectLoader.load_project(self,None,self.filepath)
        if( val  == {'FINISHED'}):
            return VISimProjectLoader.load_trajectories(self,context.object)
        return val        

def scene_visim_camera_poll(self, object):
    return object.type == 'CAMERA' and object.data.visim_cam_config.has_config == True

def scene_visim_camera_update(self, context):
    VISimProjectLoader.prepare_render(self,context)
    return None
    
def scene_visim_info_replace_files_draw(self, context):
    self.layout.label("The output folder is not empty, mixing render results is dangerous! ")

def scene_output_image_format_update(self, context):
    VISimProjectLoader.prepare_render(self,context)
    return None


def menu_func_import(self, context):
    self.layout.operator(ImportVISimProj.bl_idname, text="VISim Project (.json)")



classes = (
    ImportVISimProj,
    VISimProjectPanel,
    VISimTrajectoryReloadOperator,
    VISimProjectReloadOperator,
    VISimRenderPanel,
    VISimPrepareRenderOperator,
    VISimOGLRenderOperator,
    VISimRaytraceRenderOperator,
    VISimCameraSetting,
    VISimProjectObjectSetting,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.INFO_MT_file_import.append(menu_func_import)

    bpy.types.Object.visim_project_setting =  bpy.props.PointerProperty(type=VISimProjectObjectSetting)
    bpy.types.Camera.visim_cam_config =  bpy.props.PointerProperty(type=VISimCameraSetting)
    bpy.types.Scene.visim_render_camera = bpy.props.PointerProperty(type=bpy.types.Object,update=scene_visim_camera_update,poll=scene_visim_camera_poll,name="VISim Camera")
    bpy.types.Scene.output_image_format = bpy.props.EnumProperty(
                    name='Image Output Format',
                    description='File Format for images.',
                    items={
                    ('PNG', 'png', 'Save as png'),
                    ('OPEN_EXR_MULTILAYER', 'exr (with depth)', 'Save as multilayer exr with z')},
                    default='PNG',update=scene_output_image_format_update)


def unregister():
    bpy.types.INFO_MT_file_import.remove(menu_func_import)
    

    for cls in classes:
        bpy.utils.unregister_class(cls)
        
    del bpy.types.Object.visim_project_setting
    del bpy.types.Camera.visim_cam_config
    del bpy.types.Scene.visim_render_camera
    del bpy.types.Scene.output_image_format


if __name__ == "__main__":
    register()
