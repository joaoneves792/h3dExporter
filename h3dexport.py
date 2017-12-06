import bpy
import struct
import bmesh
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator
from mathutils import Matrix
from math import pi
from collections import OrderedDict


###############################################
# Extend Object with a is_keyframe method
def is_keyframe(ob, frame, data_path, array_index=-1):
    if ob is not None and ob.animation_data is not None and ob.animation_data.action is not None:
        for fcu in ob.animation_data.action.fcurves:
            if fcu.data_path == data_path:
                if array_index == -1 or fcu.array_index == array_index:
                    return frame in (p.co.x for p in fcu.keyframe_points)
    return False

bpy.types.Object.is_keyframe = is_keyframe
###############################################


class H3dMesh:
    def __init__(self):
        self.name = ""
        self.mesh = None
        self.obj = None
        self.vertex_groups = []
        self.animated = False
        self.blender_armature = None
        self.h3d_armature = None
        self.h3d_shape_keys = []
        self.shape_keys_original_values = {}
        self.shape_keys = []


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
        self.rotation = [0, 0, 0]  # Euler
        self.position = [0, 0, 0]
        self.index = 0
        self.keyframes = []  # These must be sorted according to frame number
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
        self.original_index = index
        
        
class H3dTriangle:
    def __init__(self, v1, v2, v3):
        self.indices = [v1, v2, v3] 

correction_matrix = Matrix.Rotation(-pi/2, 4, 'X')


def binary_write_string(f, string):
    count = len(string)
    f.write(struct.pack("<b", count))
    sb = bytes(string, 'ascii')
    final_sb = sb[:count] 
    f.write(final_sb)


def mesh_triangulate(me):
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    

def get_unique_vertices(vertices, triangles):
    helper_dict = OrderedDict()
    new_index = 0
    for vertex in vertices:
        key = "{p[0]:.3f}{p[1]:.3f}{p[2]:.3f}{n[0]:.3f}{n[0]:.3f}{n[0]:.3f}{t[0]:.3f}{t[1]:.3f}"
        key = key.format(p=vertex.position, n=vertex.normal, t=vertex.uv)
        if key not in helper_dict:
            vertex.index = new_index
            new_index += 1
            helper_dict[key] = vertex
        else:
            vertex.index = helper_dict[key].index
    
    new_vertices = list(helper_dict.values())
    
    # Update the triangle indexes
    for triangle in triangles:
        for i, vi in enumerate(triangle.indices):
            triangle.indices[i] = vertices[vi].index
            
    return new_vertices


def fill_keyframes(scene, h3d_armature):
    blender_armature = h3d_armature.blender_armature
    
    for f in range(scene.frame_start, scene.frame_end+1):
        scene.frame_set(f)
        for i, pbone in enumerate(blender_armature.pose.bones):
            if not (blender_armature.is_keyframe(f, pbone.path_from_id("location")) or
                    blender_armature.is_keyframe(f, pbone.path_from_id("rotation_axis_angle"))):
                continue
            h3d_joint = find_joint_by_name(h3d_armature, pbone.name)
            keyframe = H3dKeyframe()
            keyframe.frame = f
            matrix = blender_armature.convert_space(pose_bone=pbone, matrix=pbone.matrix,
                                                    from_space='POSE', to_space='LOCAL')
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

        # Create an ordered list and assign the parent indexes
        armature.joints = list(armature.joints_dic.values())
        for joint in armature.joints:
            for i, other_joint in enumerate(armature.joints):
                armature.joints_dic[other_joint.name].index = i
                if joint.parentName == other_joint.name:
                    joint.parentIndex = i


def find_joint_index(armature, vertex_group):
    try:
        return armature.joints_dic[vertex_group.name].index
    except KeyError:  # We might get vertex_groups belonging to key shapes (ditch those)
        return -1


def find_joint_by_name(armature, name):
    return armature.joints[armature.joints_dic[name].index]


def write_vertices(f, textual, vertices, export_uv=True, export_bones=True):
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
            if export_uv:
                line = "t {t[0]} {t[1]}\n"
                line = line.format(t=vertex.uv)
                f.write(line)
            if export_bones:
                for bone in vertex.bones:
                    line = "b {j} {w}\n"
                    line = line.format(j=bone[0], w=bone[1])
                    f.write(line)
        else:
            f.write(struct.pack("<3f", *vertex.position))
            f.write(struct.pack("<3f", *vertex.normal))
            if export_uv:
                f.write(struct.pack("<2f", *vertex.uv))
            if export_bones:
                for bone in vertex.bones:
                    f.write(struct.pack("<1i1f", *bone))


def create_vertices_list(group, num_bones=3, export_armatures=True):
    loops = group.mesh.loops
    vertices = group.mesh.vertices
    h3d_vertices = []
    for i, loop in enumerate(loops):
        vertex_groups_info = sorted(vertices[loop.vertex_index].groups, key=lambda vg: vg.weight, reverse=True)
        # Convert the vertex_group index into an index for our joint arrays
        bones_index_weight = []
        if export_armatures:
            for vg_info in vertex_groups_info:
                index = find_joint_index(group.h3d_armature, group.vertex_groups[vg_info.group])
                if -1 == index:
                    continue    # This vertex_group is not part of the armature
                weight = vg_info.weight
                bones_index_weight.append([index, weight])

        # Fill the remainder bone slots with -1
        if len(bones_index_weight) < num_bones:
            for c in range(len(bones_index_weight), num_bones):
                bones_index_weight.append([-1, 0.0])
        bones = bones_index_weight[:num_bones]

        # Normalize weights
        weight_sum = 0.0
        for bone in bones:
            weight_sum += bone[1]
        if weight_sum > 0:
            for bone in bones:
                bone[1] = bone[1]/weight_sum    
                
        if(group.mesh.uv_layers.active is not None):
            uv = (group.mesh.uv_layers.active.data[i].uv[0], 1-group.mesh.uv_layers.active.data[i].uv[1])
        else:
            uv = (0, 0)
            
        h3d_vertex = H3dVertex(vertices[loop.vertex_index].co, loop.normal, uv, bones, i)
        h3d_vertices.append(h3d_vertex)
        
    return h3d_vertices


def generate_h3d_tri_verts(group, num_bones, export_armatures, no_duplicates):
    # Get the triangles
    h3d_triangles = []
    for triangle in group.mesh.polygons:
        h3d_triangle = H3dTriangle(*triangle.loop_indices)
        h3d_triangles.append(h3d_triangle)
    # Get the vertexes
    h3d_vertices = create_vertices_list(group, num_bones, export_armatures)

    if no_duplicates:
        h3d_vertices = get_unique_vertices(h3d_vertices, h3d_triangles)
    return h3d_triangles, h3d_vertices
    

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


def group_to_h3d_mesh(scene, obj, export_armatures):
    h3d_mesh = H3dMesh()
    h3d_mesh.name = obj.name
    h3d_mesh.obj = obj
            
    h3d_mesh.vertex_groups = obj.vertex_groups
            
    world_matrix = obj.matrix_world
            
    # Temporarily disable any armature modifiers to prevent the rest pose from changing the final mesh
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
            
    if export_armatures:
        armature = obj.find_armature()
        if armature is not None:
            h3d_mesh.animated = True
            h3d_mesh.blender_armature = armature.name

    return h3d_mesh


def export_h3d(operator, file_path, textual, no_duplicates, num_bones, export_armatures, export_keyframes,
               shape_keys_behaviour):
    export_shape_keys = False
    apply_shape_keys = False

    if shape_keys_behaviour == '1':
        apply_shape_keys = False
        export_shape_keys = True
    elif shape_keys_behaviour == '2':
        apply_shape_keys = True
        export_shape_keys = False
    elif shape_keys_behaviour == '3':
        apply_shape_keys = False
        export_shape_keys = False
        
    print("running write_some_data...")
    if textual:
        f = open(file_path, 'w', encoding='utf-8')
        f.write("H3D V1\n")
    else:
        f = open(file_path, 'wb')
        f.write(struct.pack("<3c1b", bytes('H', 'ascii'), bytes('3', 'ascii'), bytes('D', 'ascii'), 1))

    scene = bpy.context.scene
    groups = []
    materials = []
    armatures = []
        
    for obj in scene.objects:
        if obj.type == 'MESH':
            shape_keys = []
            shape_key_values = {}            
            if obj.data.shape_keys is not None:
                # Set them to 0 if not applying them
                for shape_key in obj.data.shape_keys.key_blocks:
                    shape_key_values[shape_key.name] = shape_key.value
                    if not apply_shape_keys:
                        shape_key.value = 0.0
                # Set each one to 1.0 and export it as an h3d mesh
                if export_shape_keys:
                    for shape_key in obj.data.shape_keys.key_blocks:
                        shape_key.value = 1.0
                        h3d_mesh_sk = group_to_h3d_mesh(scene, obj, export_armatures)
                        shape_key.value = 0.0
                        h3d_mesh_sk.name = shape_key.name
                        shape_keys.append(h3d_mesh_sk)
                
            h3d_mesh = group_to_h3d_mesh(scene, obj, export_armatures)
            h3d_mesh.h3d_shape_keys = shape_keys
            groups.append(h3d_mesh)
            
            # Restore shape key values
            if obj.data.shape_keys is not None:
                for shape_key in obj.data.shape_keys.key_blocks:
                    shape_key.value = shape_key_values[shape_key.name]

        if obj.type == 'ARMATURE' and export_armatures:
            armature = H3dArmature()
            armature.name = obj.name
            armature.blender_armature = obj
            armatures.append(armature)
            
    # Prepare armatures and fill in keyframes
    if export_armatures:
        prepare_armatures(armatures)
        if export_keyframes:
            for armature in armatures:
                fill_keyframes(scene, armature)
                
    # assign the armatures to the correct groups
    for group in groups:
        if not group.animated:
            continue
        for armature in armatures:
            if group.blender_armature == armature.name:
                group.h3d_armature = armature

    # Write the number of groups
    if textual:
        f.write("%d\n" % len(groups))
    else:
        f.write(struct.pack("<1i", len(groups)))

    groups.sort(key=lambda g: g.name.lower());

    for group in groups:
        # Write the name of the group and its material
        if textual:
            f.write("%s\n" % group.name)
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
            
        # Now we get the triangles and vertices
        h3d_triangles, h3d_vertices = generate_h3d_tri_verts(group, num_bones, export_armatures, no_duplicates)
        
        # Write the triangles
        write_triangles(f, textual, h3d_triangles)
        # Write the vertices
        write_vertices(f, textual, h3d_vertices)

        # declare the armature
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
        
        # Shape Keys
        if textual:
            f.write("Shape keys: %d\n" % len(group.h3d_shape_keys))
        else:
            f.write(struct.pack("<1i", len(group.h3d_shape_keys)))

        for shape_key in group.h3d_shape_keys:
            sk_h3d_triangles, sk_h3d_vertices = generate_h3d_tri_verts(shape_key, num_bones=0,
                                                                       export_armatures=False, no_duplicates=False)
            
            #Elininate duplicates based on the duplicate removal from the basis
            final_sk_h3d_vertices = h3d_vertices[:]
            for i, base_vert in enumerate(h3d_vertices):
                final_sk_h3d_vertices[i] = sk_h3d_vertices[base_vert.original_index]
            
            if textual:
                f.write("%s\n" % shape_key.name)
            else:
                binary_write_string(f, shape_key.name)
            write_vertices(f, textual, final_sk_h3d_vertices, False, False)
            
    # Handle the materials
    if textual:
        f.write("%d\n" % len(materials))
    else:
        f.write(struct.pack("<1i", len(materials)))
        
    for material in materials:
        texture_image = ""
        texture = None
        if material.texture_slots[0] is not None:
            #raise Exception("Material " + material.name + "has no texture!")
            texture = material.texture_slots[0].texture
        if texture is not None:
            if hasattr(texture, 'image'):
                if texture.image.filepath is not None:
                    texture_image = bpy.path.basename(texture.image.filepath)
        
        ambient = material.ambient*(51/255)  # Hackish to say the least
        
        diffuse = list(material.diffuse_color)
        if diffuse[0] > 0.0 or diffuse[1] > 0.0 or diffuse[2] > 0.0:
            diffuse[0] = diffuse[0]*material.diffuse_intensity
            diffuse[1] = diffuse[1]*material.diffuse_intensity
            diffuse[2] = diffuse[2]*material.diffuse_intensity
        else:
            # Assume safe defaults
            diffuse = [204/255, 204/255, 204/255]
        
        specular = list(material.specular_color)
        specular[0] = specular[0]*material.specular_intensity
        specular[1] = specular[1]*material.specular_intensity
        specular[2] = specular[2]*material.specular_intensity
        emission = diffuse[:]
        emission[0] = emission[0]*material.emit
        emission[1] = emission[1]*material.emit
        emission[2] = emission[2]*material.emit
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
            line = line.format(e=emission)
            f.write(line)
            f.write("sh %f\n" % shininess)
            f.write("t %f\n" % transparency)
            
        else:
            binary_write_string(f, texture_image)
            f.write(struct.pack("<1f", ambient))
            f.write(struct.pack("<3f", *diffuse))
            f.write(struct.pack("<3f", *specular))
            f.write(struct.pack("<3f", *emission))
            f.write(struct.pack("<1f", shininess))
            f.write(struct.pack("<1f", transparency))
        
    # Write the armatures
    if textual:
        f.write("Armatures: %d\n" % len(armatures))
    else:
        f.write(struct.pack("<1i", len(armatures)))
    if export_armatures:
        for armature in armatures:
            write_armature(f, textual, armature)
    
    f.close()
    
    # Clean up
    for group in groups:
        bpy.data.meshes.remove(group.mesh)
        for shape_key in group.h3d_shape_keys:
            bpy.data.meshes.remove(shape_key.mesh)
    
    operator.report({'INFO'}, "Export Successful")
    return {'FINISHED'}


# ExportHelper is a helper class, defines filename and
# invoke() function which calls the file selector.


class Hobby3dExporter(Operator, ExportHelper):
    """Exporter to libhobby3d .h3d format"""
    bl_idname = "export_test.some_data"  # important since its how bpy.ops.import_test.some_data is constructed
    bl_label = "Export to h3d"

    # ExportHelper mixin class uses this
    filename_ext = ".h3d"

    filter_glob = StringProperty(
            default="*.h3d",
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
            
    no_duplicates = BoolProperty(
            name="Remove duplicated vertices",
            description="Eliminate vertices that share the same position, normal and uv",
            default=True,
            )

    armatures = BoolProperty(
            name="Export armatures",
            description="Export the bones of all armatures on the scene",
            default=True,
            )
    keyframes = BoolProperty(
            name="Export keyframes",
            description="Only bone location and rotation keyframes are supported",
            default=True,
            )

    num_bones = EnumProperty(
            name="Bones per vertex",
            description="How many bones can affect a vertex (Don't change this unless you modified libhobby3d)",
            items=(('1', "1", "The bone with the most weight will be picked"),
                   ('2', "2", "Pick the two heaviest bones and normalize their weights"),
                   ('3', "3", "Pick the three heaviest bones and normalize their weights"),
                   ('4', "4", "Four is the largest ammount of bones we can feed into the shader")),
            default='3',
            )

    shape_keys = EnumProperty(
            name="Shape Keys",
            description="How to handle Shape Keys",
            items=(('1', "Export",
                    "Export all Shape Keys"),
                   ('2', "Apply",
                    "Applies current shape keys transformation to base exported mesh (and doesn't export them)"),
                   ('3', "Ignore",
                    "Ignore shape keys (sets their value to 0 before exporting)")),
            default='1',
            )

    def execute(self, context):
        return export_h3d(self, self.filepath, self.textual, self.no_duplicates, int(self.num_bones), self.armatures,
                          self.keyframes, self.shape_keys)


# Only needed if you want to add into a dynamic menu
def menu_func_export(self, context):
    self.layout.operator(Hobby3dExporter.bl_idname, text="libhobby3d (.h3d)")


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
