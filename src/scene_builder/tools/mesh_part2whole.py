import sys
import os
import json

import vedo
import trimesh
import numpy as np

from transformation import translation_matrix
from transformation import euler_matrix


META_INFO_CSV = "interactive_objects.csv"
ASSET_DIR = "/home/zeyu/Workspace/Robot-CV-ROS/src/Robot-Vision-System/scene_builder/assets/interactive"
OUT_DIR = "interactive_objects"


def validate_merged_mesh(mesh_file):
    mesh = trimesh.load(mesh_file)
    mesh.show()


def load_json(json_dir):
    with open(json_dir, "r") as fjson:
        config = json.load(fjson)
    return config


def parse_3tuple(xyz):
    tkns = xyz.split(' ')

    return (float(tkns[0]), float(tkns[1]), float(tkns[2]))


def from_translation_rotation(xyz, rpy):
    trans_mat = translation_matrix(xyz)
    rot_mat = euler_matrix(*rpy)
    
    return np.dot(trans_mat, rot_mat)


def load_mesh(mesh_dir, xyz="0 0 0", rpy="0 0 0", pxyz="0 0 0", prpy="0 0 0"):
    mesh = trimesh.load(mesh_dir)

    tf = from_translation_rotation(parse_3tuple(xyz), parse_3tuple(rpy))
    mesh.apply_transform(tf)

    tf = from_translation_rotation(parse_3tuple(pxyz), parse_3tuple(prpy))
    mesh.apply_transform(tf)

    return mesh


def clean_up(config_dir, mesh_dir):
    config = load_json(config_dir)

    dst_dir = "{}/{}/".format(ASSET_DIR, config["object_name"])

    os.system("cp {} {}".format(config_dir, dst_dir))
    os.system("cp {}/{}.obj {}".format(OUT_DIR, config["object_name"], dst_dir))
    

def part2whole(config_dir):
    config = load_json(config_dir)
    asset_dir = config["asset_dir"]

    scene = trimesh.Scene()

    for p in config["parts"]:
        mesh_dir = "{}/{}".format(asset_dir, p["mesh_dir"])
        mesh = load_mesh(
            mesh_dir,
            p["xyz"],
            p["rpy"],
            p["pxyz"],
            p["prpy"],
        )

        scene.add_geometry(mesh)
    
    scene.show()

    # save merged parts into a single file
    file_out = "{}/{}.obj".format(OUT_DIR, config["object_name"])

    whole_mesh = scene.dump(concatenate=True)
    whole_mesh.export(file_out)
    print("[INFO] Merged mesh is saved at: {}".format(file_out))

    with open("{}/{}".format(OUT_DIR, META_INFO_CSV), "a") as fin:
        line = "{},{}\n".format(config["object_name"], config["category"])
        fin.write(line)

    # validate merged parts
    validate_merged_mesh(file_out)
    clean_up(config_dir, file_out)



if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Need to specify the config JSON file directory")
        exit(1)

    # validate_merged_mesh("cabinet_0007.obj")
    part2whole(sys.argv[1])

