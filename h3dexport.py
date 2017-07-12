import bpy
import struct
from mathutils import Matrix
from math import pi
from collections import namedtuple


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


#K-D Tree######################################
class KDNode(namedtuple('KDNode', 'vertex left_child right_child')):
    pass

#Pass a copy of the vertices list since it is going to be sorted a lot of times
def kdtree(vertices, depth=0):
    k = 3 #3D
    axis = depth % k
    
    if len(vertices) == 0:
        return None
    
    #TODO instead of sorting pick 10% random values and calculate their median
    vertices.sort(key=lambda v:v.position[axis], reverse=False)
    median = len(vertices) // 2
    
    return KDNode(vertex=vertices[median],
            left_child=kdtree(vertices[:median], depth+1),
            right_child=kdtree(vertices[median + 1:], depth+1) )
            
def is_near(f1, f2):
    return ( abs(f1-f2) < 0.01)

def is_similar(v1, v2):
    return (is_near(v1.position[0], v2.position[0]) and
            is_near(v1.position[1], v2.position[1]) and
            is_near(v1.position[2], v2.position[2]) and
            is_near(v1.normal[0], v2.normal[0]) and
            is_near(v1.normal[1], v2.normal[1]) and
            is_near(v1.normal[2], v2.normal[2]) and
            is_near(v1.uv[0], v2.uv[0]) and
            is_near(v1.uv[1], v2.uv[1]) )

def squared_distance(v1, v2):
    return  ( (v1.position[0]-v2.position[0])**2 ) + ( (v1.position[1]-v2.position[1])**2 ) + ( (v1.position[2]-v2.position[2])**2 )

def mark_if_similar(vertex, other_vertex):
    if is_similar(vertex, other_vertex):
        other_vertex.similar_index = vertex.similar_index

def identify_similars(vertex, node, depth=0, best=None):
    axis = depth % 3
         
    if node is None:
        return best
    if best is None:
        best = node
 
    # consider the current node
    if squared_distance(node.vertex, vertex) < squared_distance(best.vertex, vertex):
        best = node
 
    diff = vertex.position[axis] - node.vertex.position[axis]
    close, away = (node.left_child, node.right_child) if diff <= 0 else (node.right_child, node.left_child)
 
    # search the near branch
    best = identify_similars(vertex, close, depth+1, best)
    if best.vertex.similar_index < 0:
        mark_if_similar(vertex, best.vertex)
    
    # search the away branch - maybe
    if diff**2 < squared_distance(best.vertex, vertex):
        best = identify_similars(vertex, away, depth+1, best)
        if best.vertex.similar_index < 0:
            mark_if_similar(vertex, best.vertex)
  
    return best

def get_unique_vertices(vertices, triangles, kdtree_root):
    for vertex in vertices:
        if vertex.similar_index < 0:
            vertex.similar_index = vertex.index #Mark this vertex as unique
            identify_similars(vertex, kdtree_root)

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


class H3dVertex:
    def __init__(self, position, normal, uv, bones, index):
        self.position = position
        self.normal = normal
        self.uv = uv
        self.bones = bones
        self.index = index
        self.similar_index = -1
        
        
class H3dTriangle:
    def __init__(self, v1, v2, v3):
        self.indices = [v1, v2, v3]

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

def write_vertices(f, textual, vertices):
    if textual:
        f.write("%d\n" % len(vertices))
    else:
        f.write(struct.pack("<1i", len(vertices)))
    
    for vertex in vertices:
        if textual:
            line = "v {v.x} {v.y} {v.z}\n"
            line = line.format(v=vertex.position)
            f.write(line)
            line = "n {n.x} {n.y} {n.z}\n"
            line = line.format(n=vertex.normal)
            f.write(line)
            line = "t {t[0]} {t[1]}\n"
            line = line.format(t=vertex.uv)
            f.write(line)
            line = "b {j} {w}\n"
            line = line.format(j=vertex.bones[0], w=vertex.bones[1])
            f.write(line)
        else:
            f.write(struct.pack("<3f", *vertex.position))
            f.write(struct.pack("<3f", *vertex.normal))
            f.write(struct.pack("<2f", *vertex.uv))
            f.write(struct.pack("<1i1f", *vertex.bones))


def create_vertices_list(group):
    loops = group.mesh.loops
    vertices = group.mesh.vertices
    h3d_vertices = []
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
        
        uv = (group.mesh.uv_layers.active.data[i].uv[0], 1-group.mesh.uv_layers.active.data[i].uv[1])
        bones = (bones_index_weight[0][0], bones_index_weight[0][1])

        h3d_vertex = H3dVertex(vertices[loop.vertex_index].co, loop.normal, uv, bones, i)
        h3d_vertices.append(h3d_vertex)
        
    return h3d_vertices

def write_triangles(f, textual, h3d_triangles):
    if textual:
        f.write("%d\n" % len(h3d_triangles))
    else:
        f.write(struct.pack("<1i", len(h3d_triangles)))
                
    for triangle in h3d_triangles:
        if textual:
            line = "tri {l[0]} {l[1]} {l[2]}\n"
            line = line.format(l=triangle.indices)
            f.write(line)
        else:
            f.write(struct.pack("<3i", *triangle.indices))


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
            
            #Temporarily disable any armature modifiers to prevent the rest pose from changing the final mesh
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
            
    #Prepare armatures and fill in keyframes
    prepare_armatures(armatures)
    for armature in armatures:
        fill_keyframes(scene, armature)
    #assign the armatures to the correct groups        
    for group in groups:
        if not group.animated:
            continue
        for armature in armatures:
            if group.blender_armature == armature.name:
                group.h3d_armature = armature

    #Write the number of groups    
    if textual:
        f.write("%d\n" % len(groups))
    else:
        f.write(struct.pack("<1i", len(groups)))

    
    for group in groups:
        #Write the name of the group and its material
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
            

        #Get the triangles
        h3d_triangles = []
        for triangle in group.mesh.polygons:
            h3d_triangle = H3dTriangle(*triangle.loop_indices)
            h3d_triangles.append(h3d_triangle)
        #Get the vertexes
        h3d_vertices = create_vertices_list(group)
        
        vertex_tree = kdtree(h3d_vertices[:])        
        get_unique_vertices(h3d_vertices, h3d_triangles, vertex_tree)
        
        
        #Write the triangles
        write_triangles(f, textual, h3d_triangles)
        #Write the vertices
        write_vertices(f, textual, h3d_vertices)



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
