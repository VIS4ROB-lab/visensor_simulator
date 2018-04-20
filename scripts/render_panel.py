import bpy


# create a camera_setting in the type
# create a global property about selected camera

class VISimRenderOperator(bpy.types.Operator):
    bl_idname = "visim.render"
    bl_label = "Render Camera"

    def execute(self, context):
        print("Hello World")
        return {'FINISHED'}




class VISimRenderPanel(bpy.types.Panel):
    bl_idname = "RENDER_PT_visim_render_panel"
    bl_label = "VISim Render"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"

    def draw(self, context):    	
        self.layout.label(text="Asdawed asd")
        self.layout.operator(VISimRenderOperator.bl_idname)
        self.layout.prop(context.scene, "visim_camera", expand=True)

class VISimCameraSetting(bpy.types.PropertyGroup):
	has_config = bpy.props.BoolProperty(
        name="has visim configuration",
        description="Indicates if this camera,\n"
                    " has a VISim configuration",
        default=False
    )

# Register
classes = ( VISimRenderOperator, VISimRenderPanel , VISimCameraSetting )


def scene_visim_camera_poll(self, object):
    return object.mysetting.has_config == True

def scene_visim_camera_update(self, context):
	return None
    

def register():	
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Camera.visim_config =  bpy.props.PointerProperty(type=VISimCameraSetting) 
    bpy.types.Scene.visim_camera = bpy.props.PointerProperty(type=bpy.types.Camera,update=scene_visim_camera_update,poll=scene_visim_camera_poll)


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
# 	RegisterWowPortalPlaneProperties()
# 	bpy.utils.register_class(WowPortalPlanePanel)

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