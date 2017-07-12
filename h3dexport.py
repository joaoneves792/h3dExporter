import bpy
import struct
from mathutils import Matrix
from math import pi

###############################################
#Extend Object with a is_keyframe method
def is_keyframe(ob, frame, data_path, array_index=-1):
    if ob is not None and ob.animation_data is not None and ob.animation_data.action is not None:
        for fcu in ob.animation_data.action.fcurves:
            if fcu.data_path == data_path:
                if array_index == -1 or fcu.array_index == array_index:
                    return frame in (p.co.x for p in fcu.keyframe_points)
    return False

bpy.types.Object.is_keyframe = is_keyframe
###############################################


correction_matrix = Matrix.Rotation(-pi/2, 4, 'X')

def binary_write_string(f, string):
    count = len(string)
    f.write(struct.pack("<b", count))
    sb = bytes(string, 'ascii')
    final_sb = sb[:count] 
    f.write(final_sb)

def mesh_triangulate(me):
    import bmesh
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    
def matrix_difference(mat_src, mat_dst):
    mat_dst_inv = mat_dst.inverted()
    return mat_dst_inv * mat_src

#def joint_correction(value):
#    return (value[2], value[1], -value[0])


#def joint_correction(value):
#    return (value[1], value[0], value[1])

#def joint_correction(value):
#    return (-value[0], value[2], value[1])

def joint_correction(value):
    return (-value[0], value[2], value[1])


class H3dMesh:
    def __init__(self):
        self.mesh = None
        self.vertex_groups = []
        self.animated = False
        self.blender_armature = None
        self.h3d_armature = None

class H3dKeyframe:
    def __init__(self):
        self.frame = 0
        self.rotation = [0, 0, 0]
        self.position = [0, 0, 0]

class H3dJoint:
    def __init__(self):
        self.name = None
        self.parentName = None
        self.parentIndex = -1
        self.matrix = None
        self.rotation = [0,0,0] #Euler
        self.position = [0,0,0]
        self.index = 0
        self.keyframes = [] #These must be sorted according to frame number
        self.blender_bone = None

class H3dArmature:
    def __init__(self):
        self.name = ""
        self.blender_armature = None
        self.joints_dic = {}
        self.joints = []

def fill_keyframes(scene, h3d_armature):
    base_bone_correction = Matrix.Rotation(pi / 2, 4, 'Z')
    blender_armature = h3d_armature.blender_armature        
    
    for f in range(scene.frame_start, scene.frame_end+1):
        scene.frame_set(f)
        for i, pbone in enumerate(blender_armature.pose.bones):
            if not (blender_armature.is_keyframe(f, pbone.path_from_id("location")) or blender_armature.is_keyframe(f, pbone.path_from_id("rotation_axis_angle"))):
                continue
            h3d_joint = find_joint_by_name(h3d_armature, pbone.name)
            keyframe = H3dKeyframe()
            keyframe.frame = f
            matrix = blender_armature.convert_space(pose_bone=pbone, matrix=pbone.matrix, from_space='POSE', to_space='LOCAL')
            keyframe.position = matrix.to_translation()
            keyframe.rotation = matrix.to_euler("XYZ")
            #keyframe.rotation = (keyframe.rotation[0], keyframe.rotation[1], -keyframe.rotation[2])
 
            h3d_joint.keyframes.append(keyframe)

def write_armature(f, textual, armature):
    if textual:
        f.write("%s\n" % armature.name)
        f.write("Joint count: %d\n" % len(armature.joints))
    else:
        binary_write_string(f, armature.name)
        f.write(struct.pack("<1i", len(armature.joints)))
        
    for joint in armature.joints:
        if textual:
            f.write("%s\n" % joint.name)
            f.write("p %f %f %f\n" % (joint.position[0], joint.position[1], joint.position[2]))
            f.write("r %f %f %f\n" % (joint.rotation[0], joint.rotation[1], joint.rotation[2]))
            f.write("< %d\n" % joint.parentIndex)
            f.write("Keyframes: %d\n" % len(joint.keyframes))
            for keyframe in joint.keyframes:
                f.write("f %d\n" % keyframe.frame)
                f.write("p %f %f %f\n" % (keyframe.position[0], keyframe.position[1], keyframe.position[2]))
                f.write("r %f %f %f\n" % (keyframe.rotation[0], keyframe.rotation[1], keyframe.rotation[2]))

        else:
            binary_write_string(f, joint.name)
            f.write(struct.pack("<3f", *joint.position))
            f.write(struct.pack("<3f", *joint.rotation))
            f.write(struct.pack("<1i", joint.parentIndex))
            f.write(struct.pack("<1i", len(joint.keyframes)))
            for keyframe in joint.keyframes:
                f.write(struct.pack("<1i3f3f", keyframe.frame, *keyframe.position, *keyframe.rotation))

def prepare_armatures(armatures_list):
    for armature in armatures_list:
        base_matrix = armature.blender_armature.matrix_basis
        bones = list(armature.blender_armature.data.bones)
        for bone in bones:
            joint = H3dJoint()
            joint.name = bone.name
            parent_bone = bone.parent
            joint.blender_bone = bone
            if parent_bone is not None:
                joint.parentName = parent_bone.name
            else:
                joint.parentName = ""
            joint.matrix = correction_matrix * base_matrix * bone.matrix_local


            #joint.position = joint.matrix.to_translation()
            #joint.rotation = joint_correction(joint.matrix.to_euler("XZY"))
            joint.position = joint.matrix.to_translation()
            joint.rotation = joint.matrix.to_euler("XYZ")

            armature.joints_dic[joint.name] = joint

        #Create an oredered list and assign the parent indexes        
        armature.joints = list(armature.joints_dic.values())
        for joint in armature.joints:
            for i, other_joint in enumerate(armature.joints):
                armature.joints_dic[other_joint.name].index = i
                if(joint.parentName == other_joint.name):
                    joint.parentIndex = i
                    
def find_joint_index(armature, vertex_group):
    try:
        return armature.joints_dic[vertex_group.name].index
    except KeyError: #We might get vertex_groups belonging to key shapes (ditch those)
        return -1
    
def find_joint_by_name(armature, name):
    return armature.joints[armature.joints_dic[name].index]

def write_some_data(context, filepath, textual):
        
    print("running write_some_data...")
    if textual:
        f = open(filepath, 'w', encoding='utf-8')
        f.write("H3D V1\n")
    else:
        f = open(filepath, 'wb')
        f.write(struct.pack("<3c1b", bytes('H', 'ascii'), bytes('3', 'ascii'), bytes('D', 'ascii'), 1))

    scene = bpy.context.scene
    groups = []
    materials = []
    armatures = []
        
    for obj in scene.objects:
        if obj.type == 'MESH':
            h3d_mesh = H3dMesh()
            
            h3d_mesh.vertex_groups = obj.vertex_groups
            
            world_matrix = obj.matrix_world
            
            for modifier in obj.modifiers:
                if modifier.type == 'ARMATURE':
                    modifier.show_render = False
            
            mesh = obj.to_mesh(scene, True, 'RENDER')
            
            for modifier in obj.modifiers:
                if modifier.type == 'ARMATURE':
                    modifier.show_render = True
                    
            mesh.transform(correction_matrix*world_matrix)
            mesh_triangulate(mesh)
            mesh.calc_normals_split()
            h3d_mesh.mesh = mesh
            
            armature = obj.find_armature()
            if armature is not None:
                h3d_mesh.animated = True
                h3d_mesh.blender_armature = armature.name
                                
            groups.append(h3d_mesh)
            
        if obj.type == 'ARMATURE':
            armature = H3dArmature()
            armature.name = obj.name
            armature.blender_armature = obj
            armatures.append(armature)
            
    prepare_armatures(armatures)
    
    for armature in armatures:
        fill_keyframes(scene, armature)
        
    for group in groups:
        if not group.animated:
            continue
        for armature in armatures:
            if group.blender_armature == armature.name:
                group.h3d_armature = armature

    
    if textual:
        f.write("%d\n" % len(groups))
    else:
        f.write(struct.pack("<1i", len(groups)))
    
    for group in groups:
        if textual:
            f.write("%s\n" % group.mesh.name)
        else:
            binary_write_string(f, group.mesh.name)

        material = group.mesh.materials[0]
        material = bpy.data.materials[material.name]
        if material is None:
            material_index = -1
        else:
            if material not in materials:
                materials.append(material)
                material_index = len(materials)-1
            else:
                material_index = materials.index(material)
                
        if textual:
            f.write("%d\n" % material_index)
        else:
            f.write(struct.pack("<1i", material_index))
            
        vertices = group.mesh.vertices
        if textual:
            f.write("%d\n" % len(group.mesh.polygons))
        else:
            f.write(struct.pack("<1i", len(group.mesh.polygons)))
            
        for triangle in group.mesh.polygons:
            if textual:
                line = "tri {l[0]} {l[1]} {l[2]}\n"
                line = line.format(l=triangle.loop_indices)
                f.write(line)
            else:
                f.write(struct.pack("<3i", *triangle.loop_indices))
        
        loops = group.mesh.loops
        if textual:
            f.write("%d\n" % len(loops))
        else:
            f.write(struct.pack("<1i", len(loops)))
    
        for i, loop in enumerate(loops):
            vertex_groups_info = sorted(vertices[loop.vertex_index].groups, key=lambda vg:vg.weight, reverse=True) #TODO Dont forget to normalize the weights (sum must be 1)
            #Convert the vertex_group index into an index for our joint arrays
            bones_index_weight = []
            for vg_info in vertex_groups_info:
                    index = find_joint_index(group.h3d_armature, group.vertex_groups[vg_info.group])
                    if -1 == index:
                        continue    #This vertex_group is not part of the armature
                    weight = vg_info.weight
                    bones_index_weight.append((index, weight))
                    
            #Fill the remainder bone slots with -1
            if len(vertex_groups_info) < 3:
                for c in range(len(vertex_groups_info), 3):
                    bones_index_weight.append((-1, 0.0))
            
            if textual:
                line = "v {v.x} {v.y} {v.z}\n"
                line = line.format(v=vertices[loop.vertex_index].co)
                f.write(line)
                line = "n {n.x} {n.y} {n.z}\n"
                line = line.format(n=loop.normal)
                f.write(line)
                line = "t {u} {v}\n"
                line = line.format(u=group.mesh.uv_layers.active.data[i].uv[0], v=1-group.mesh.uv_layers.active.data[i].uv[1])
                f.write(line)
                line = "b {j} {w}\n"
                line = line.format(j=bones_index_weight[0][0], w=bones_index_weight[0][1])
                f.write(line)

            else:
                f.write(struct.pack("<3f", *vertices[loop.vertex_index].co))
                f.write(struct.pack("<3f", *loop.normal))
                f.write(struct.pack("<2f", group.mesh.uv_layers.active.data[i].uv[0], 1-group.mesh.uv_layers.active.data[i].uv[1]))
                f.write(struct.pack("<1i1f", *bones_index_weight[0]))

        #declare the armature                    
        if not group.animated:
            if textual:
                f.write("Animated:False\n")
            else:
                f.write(struct.pack("<1b", 0))
        else:
            if textual:
                f.write("Animated:True\n")
                f.write("%s\n" % group.h3d_armature.name)
            else:
                f.write(struct.pack("<1b", 1))
                binary_write_string(f, group.h3d_armature.name)

        
    #Handle the materials
    if textual:
        f.write("%d\n" % len(materials))
    else:
        f.write(struct.pack("<1i", len(materials)))
        
    for material in materials:
        texture_image = ""
        texture = material.texture_slots[0].texture
        if texture is not None:
            if texture.image is not None:
                if texture.image.filepath is not None:
                    texture_image = bpy.path.basename(texture.image.filepath)
        
        ambient = material.ambient*(51/255) #Hackish to say the least
        
        diffuse = list(material.diffuse_color)
        if(diffuse[0]>0.0 or diffuse[1]>0.0 or diffuse[2]>0.0):
            diffuse[0] = diffuse[0]*material.diffuse_intensity
            diffuse[1] = diffuse[1]*material.diffuse_intensity
            diffuse[2] = diffuse[2]*material.diffuse_intensity
        else:
            #Assume safe defaults
            diffuse = [204/255, 204/255, 204/255]
        
        specular = list(material.specular_color)
        specular[0] = specular[0]*material.specular_intensity
        specular[1] = specular[1]*material.specular_intensity
        specular[2] = specular[2]*material.specular_intensity
        emissive = diffuse[:]
        emissive[0] = emissive[0]*material.emit
        emissive[1] = emissive[1]*material.emit
        emissive[2] = emissive[2]*material.emit
        shininess = material.specular_intensity*128.0
        transparency = material.alpha
        
        if textual:
            f.write("%s\n" % texture_image)
            f.write("a %f\n" % ambient)
            line = "d {d[0]} {d[1]} {d[2]}\n"
            line = line.format(d=diffuse)
            f.write(line)
            line = "s {s[0]} {s[1]} {s[2]}\n"
            line = line.format(s=specular)
            f.write(line)
            line = "e {e[0]} {e[1]} {e[2]}\n"
            line = line.format(e=emissive)
            f.write(line)
            f.write("sh %f\n" % shininess)
            f.write("t %f\n" % transparency)
            
        else:
            binary_write_string(f, texture_image)
            f.write(struct.pack("<1f", ambient))
            f.write(struct.pack("<3f", *diffuse))
            f.write(struct.pack("<3f", *specular))
            f.write(struct.pack("<3f", *emissive))
            f.write(struct.pack("<1f", shininess))
            f.write(struct.pack("<1f", transparency))
        
    #Write the armatures
    if textual:
        f.write("Armatures: %d\n" % len(armatures))
    else:
        f.write(struct.pack("<1i", len(armatures)))
    for armature in armatures:
        write_armature(f, textual, armature)
    
    f.close()
    
    #Clean up
    for group in groups:
        bpy.data.meshes.remove(group.mesh)
    
    return {'FINISHED'}


# ExportHelper is a helper class, defines filename and
# invoke() function which calls the file selector.
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator


class Hobby3dExporter(Operator, ExportHelper):
    """Exporter to libhobby3d .h3d format"""
    bl_idname = "export_test.some_data"  # important since its how bpy.ops.import_test.some_data is constructed
    bl_label = "Export to h3d"

    # ExportHelper mixin class uses this
    filename_ext = ".h3d"

    filter_glob = StringProperty(
            default="*.txt",
            options={'HIDDEN'},
            maxlen=255,  # Max internal buffer length, longer would be clamped.
            )

    # List of operator properties, the attributes will be assigned
    # to the class instance from the operator settings before calling.
    textual = BoolProperty(
            name="Write text",
            description="Output text (for debugging)",
            default=False,
            )

    type = EnumProperty(
            name="Example Enum",
            description="Choose between two items",
            items=(('OPT_A', "First Option", "Description one"),
                   ('OPT_B', "Second Option", "Description two")),
            default='OPT_A',
            )

    def execute(self, context):
        return write_some_data(context, self.filepath, self.textual)


# Only needed if you want to add into a dynamic menu
def menu_func_export(self, context):
    self.layout.operator(ExportSomeData.bl_idname, text="libhobby3d (.h3d)")


def register():
    bpy.utils.register_class(Hobby3dExporter)
    bpy.types.INFO_MT_file_export.append(menu_func_export)


def unregister():
    bpy.utils.unregister_class(Hobby3dExporter)
    bpy.types.INFO_MT_file_export.remove(menu_func_export)


if __name__ == "__main__":
    register()

    # test call
    bpy.ops.export_test.some_data('INVOKE_DEFAULT')
