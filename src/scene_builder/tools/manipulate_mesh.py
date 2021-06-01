import trimesh

from transformation import from_translation_rotation


def view_mesh(mesh_file):
    mesh = trimesh.load(mesh_file)
    mesh.show()


def view_scene(mesh_files):
    scene = trimesh.Scene()

    for file in mesh_files:
        mesh = trimesh.load(file)
        scene.add_geometry(mesh)

    scene.show()


def set_mesh_origin(mesh, translation, rotation):
    mesh = mesh.copy()

    tf_mat = from_translation_rotation(translation, rotation)
    mesh.apply_transform(tf_mat)

    return mesh


def align_mesh(mesh):
    center = mesh.center_mass
    
    # set_mesh_origin will make a deep copy of the input mesh
    mesh = set_mesh_origin(mesh, -center, [0, 0, 0, 1])
    
    return mesh


def save_mesh(mesh, outfile):
    mesh.export(outfile)
    print("Mesh is saved at: {}".format(outfile))


def foo(mesh_file):
    mesh = trimesh.load(mesh_file)
    # mesh.apply_scale([1, 1, 0.94])

    mesh.show()
    # save_mesh(mesh, "none_motion.obj")




if __name__ == "__main__":
    # view_mesh(mesh_file)
    # view_scene(mesh_files)
    mesh_file = "/home/zeyu/Workspace/Robot-CV-ROS/src/Robot-Vision-System/scene_builder/assets/interactive/fridge_0003/none_motion.obj"
    foo(mesh_file)