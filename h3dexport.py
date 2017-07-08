import bpy
import struct

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

def write_some_data(context, filepath, textual):
        
    print("running write_some_data...")
    if textual:
        f = open(filepath, 'w', encoding='utf-8')
        f.write("H3D V1")
    else:
        f = open(filepath, 'wb')
        f.write(struct.pack("<3c1b", bytes('H', 'ascii'), bytes('3', 'ascii'), bytes('D', 'ascii'), 1))

    scene = bpy.context.scene
    groups = []
    materials = []
        
    for obj in scene.objects:
        if obj.type == 'MESH':
            world_matrix = obj.matrix_world
            group = obj.to_mesh(scene, True, 'RENDER')
            group.transform(world_matrix)
            mesh_triangulate(group)
            group.calc_normals_split()
            groups.append(group)
    if textual:
        f.write("%d\n" % len(groups))
    else:
        f.write(struct.pack("<1i", len(groups)))
    
    for group in groups:
        if textual:
            f.write("%s\n" % group.name)
        else:
            binary_write_string(f, group.name)

        material = group.materials[0]
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
            
        vertices = group.vertices
        if textual:
            f.write("%d\n" % len(group.polygons))
        else:
            f.write(struct.pack("<1i", len(group.polygons)))
            
        for triangle in group.polygons:
            if textual:
                line = "tri {l[0]} {l[1]} {l[2]}\n"
                line = line.format(l=triangle.loop_indices)
                f.write(line)
            else:
                f.write(struct.pack("<3i", *triangle.loop_indices))
        
        loops = group.loops
        if textual:
            f.write("%d\n" % len(loops))
        else:
            f.write(struct.pack("<1i", len(loops)))
    
        for i, loop in enumerate(loops):
            if textual:
                line = "v {v.x} {v.y} {v.z}\n"
                line = line.format(v=vertices[loop.vertex_index].co)
                f.write(line)
                line = "n {n.x} {n.y} {n.z}\n"
                line = line.format(n=loop.normal)
                f.write(line)
                line = "t {u} {v}\n"
                line = line.format(u=group.uv_layers.active.data[i].uv[0], v=1-group.uv_layers.active.data[i].uv[1])
                f.write(line)
            else:
                f.write(struct.pack("<3f", *vertices[loop.vertex_index].co))
                f.write(struct.pack("<3f", *loop.normal))
                f.write(struct.pack("<2f", group.uv_layers.active.data[i].uv[0], 1-group.uv_layers.active.data[i].uv[1]))

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
        
        ambient = material.ambient*51 #Hackish to say the least
        
        diffuse = list(material.diffuse_color)
        if(diffuse[0]>0.0 or diffuse[1]>0.0 or diffuse[2]>0.0):
            diffuse[0] = diffuse[0]*material.diffuse_intensity
            diffuse[1] = diffuse[1]*material.diffuse_intensity
            diffuse[2] = diffuse[2]*material.diffuse_intensity
        else:
            #Assume safe defaults
            diffuse = [204/255, 204/255, 204/255]
        
        specular = list(material.specular_color)
        specular[0] = specular[0]*material.specular_intensity*255
        specular[1] = specular[1]*material.specular_intensity*255
        specular[2] = specular[2]*material.specular_intensity*255
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
        
    
    f.close()
    
    #Clean up
    for group in groups:
        bpy.data.meshes.remove(group)
    
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
