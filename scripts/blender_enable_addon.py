import bpy
bpy.ops.wm.addon_install(filepath='/home/shane/Downloads/testaddon.py')
bpy.ops.wm.addon_enable(module='blender_visensor_sim_tool_lite')
bpy.ops.wm.save_userpref()
