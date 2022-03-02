import os

from xacro_ros import XacroROS
from transformation import euler_from_quaternion

from global_settings import MAIN_XACRO_FILENAME
from global_settings import RIGIT_OBJ_XACRO_FILENAME
from global_settings import RIGID_OBJ_XACRO_MACRO_NAME
from global_settings import RIGID_MESH_DIR_PREFIX
from global_settings import RIGID_DEFAULT_XYZ, RIGID_DEFAULT_RPY
from global_settings import RIGID_DEFAULT_JOINT_TYPE, RIGID_DEFAULT_SCALE
from global_settings import XACRO_INC_PREFIX, XACRO_INC_INTERACTIVE_PREFIX
from global_settings import BACKGROUND_MESH_DIR_PREFIX, BACKGROUND_MESH_FOLDER
from global_settings import INTERACTIVE_CATEGORY
from global_settings import BACKGROUND_CATEGORY


class XacroScene(object):
    """
    A simulation scene maintains 2 xacro files
        - main.xacro: will include and create 1 rigid_object.xacro 
            and n interactive_object.xacro
        - rigid_object.xacro: will specify all rigid-body objects in
            the xacro file
    """


    def __init__(self, scene_name, output_dir):
        """
        Constructor

        @param scene_name (string): the name of the scene
        @param output_name (string): the output directory of the ROS scene_builder package
        """
        self.scene_name_ = scene_name
        self.output_dir_ = output_dir

        # output directory of the scene
        self.output_dst_ = "{}/{}".format(output_dir, scene_name)

        # store background mesh filenames
        self.bgm_files_ = []


        self.main_xacro_ = self.create_main_xacro_(scene_name)
        self.rigid_obj_xacro_ = self.create_rigid_obj_xacro_()

        # include rigid object xacro in main
        self.main_xacro_.add_include("{}/{}/{}".format(XACRO_INC_PREFIX, scene_name, RIGIT_OBJ_XACRO_FILENAME))
        self.main_xacro_.instantiate_macro(RIGID_OBJ_XACRO_MACRO_NAME)

    
    def __str__(self):
        ret = ""

        ret += "* [INFO] Filename: main.xacro\n"
        ret += self.main_xacro_.__str__() + "\n"

        ret += "* [INFO] Filename: rigid_objects.xacro\n"
        ret += self.rigid_obj_xacro_.__str__() + "\n"

        return ret


    def save(self):
        """
        Save current scene into organized xacro files

        @return (string): return the output destination if xacro is 
            successfully saved, otherwise return an exampty string ""
        """
        # check if the output directory already exists
        if os.path.isdir(self.output_dst_):
            ret = input("Output directory `{}` is already exist.\nReplace it? [y/n]".format(self.output_dst_))
            if ret in ["y", "Y"]:
                os.system("rm -r {}".format(self.output_dst_))
            else:
                print("Scene builder exit")
                return ""

        # create the output directory
        try:
            os.system("mkdir {}".format(self.output_dst_))
            os.system("mkdir {}/{}".format(self.output_dst_, BACKGROUND_MESH_FOLDER))
        except:
            print("[ERROR] `{}` does not exist".format(self.output_dst_))
            return ""
        
        self.main_xacro_.save("{}/{}".format(self.output_dst_, MAIN_XACRO_FILENAME))
        self.rigid_obj_xacro_.save("{}/{}".format(self.output_dst_, RIGIT_OBJ_XACRO_FILENAME))

        print("[INFO] xacro scene is save at: {}".format(self.output_dst_))
        return self.output_dst_


    def get_bgm_files(self):
        """
        Get background mesh filenames

        @return (list of string): a list of background filenames
        """
        return self.bgm_files_[:]


    def add(self, target, parent=None):
        """
        Add a new node into current xacro scene

        @param target (igraph.Vertex): target node to be added
        @param parent (igraph.Vertex): parent node (if exists) of the target node
        """
        if target["type"] == "ConceptNode" and target["label"] not in BACKGROUND_CATEGORY:
            self.add_concept_link_(target, parent)
        else:
            if target["label"] in BACKGROUND_CATEGORY:
                self.add_background_object_(target, parent)
            elif target["label"] in INTERACTIVE_CATEGORY:
                self.add_interactive_object_(target, parent)
            else:
                self.add_rigid_object_(target, parent)


    def add_background_object_(self, target, parent):
        """
        Add a background object into the xacro scene

        Background object only have visual effect, did not have any 
        collision detection or interaction capability.

        @param target (igraph.Vertex): target node to be added
        @param parent (igraph.Vertex): parent node (if exists) of the target node
        """
        link_name = self.get_link_name_(target)
        mesh_dir = self.get_bg_mesh_dir_(target)
        scale = 1.0 if target["scale"] is None else target["scale"]

        # Add object link into the xacro
        # the <xyz> and <rpy> specify the origin of the mesh
        self.rigid_obj_xacro_.add_background(
            link_name=link_name,
            mesh_dir=mesh_dir,
            xyz=RIGID_DEFAULT_XYZ,
            rpy=RIGID_DEFAULT_RPY,
            scale=[scale] * 3,
            ns=RIGID_OBJ_XACRO_MACRO_NAME
        )

        # noted: target["position"] and target["orientation"] defines
        # the transform between target and its parent
        if parent is not None:
            self.rigid_obj_xacro_.add_joint(
                joint_name=self.get_joint_name_(target, parent),
                joint_type=RIGID_DEFAULT_JOINT_TYPE,
                child_link=link_name,
                parent_link=self.get_link_name_(parent),
                xyz=target["position"],
                rpy=euler_from_quaternion(target["orientation"], "sxyz"),
                ns=RIGID_OBJ_XACRO_MACRO_NAME
            )
        
        # record background mesh filename
        self.bgm_files_.append("{}/{}/{}.ply".format(
            self.output_dst_, BACKGROUND_MESH_FOLDER, target["id"])
        )

    
    def add_rigid_object_(self, target, parent):
        """
        Add a rigid object into the xacro scene

        @param target (igraph.Vertex): target node to be added
        @param parent (igraph.Vertex): parent node (if exists) of the target node
        """
        link_name = self.get_link_name_(target)
        mesh_dir = self.get_mesh_dir_(target)

        # Add object link into the xacro
        # the <xyz> and <rpy> specify the origin of the mesh
        self.rigid_obj_xacro_.add_object(
            link_name=link_name,
            mesh_dir=mesh_dir,
            xyz=RIGID_DEFAULT_XYZ,
            rpy=RIGID_DEFAULT_RPY,
            scale=[target["scale"], target["scale"], target["scale"]],
            ns=RIGID_OBJ_XACRO_MACRO_NAME
        )

        # noted: target["position"] and target["orientation"] defines
        # the transform between target and its parent
        if parent is not None:
            self.rigid_obj_xacro_.add_joint(
                joint_name=self.get_joint_name_(target, parent),
                joint_type=RIGID_DEFAULT_JOINT_TYPE,
                child_link=link_name,
                parent_link=self.get_link_name_(parent),
                xyz=target["position"],
                rpy=euler_from_quaternion(target["orientation"], "sxyz"),
                ns=RIGID_OBJ_XACRO_MACRO_NAME
            )


    def add_interactive_object_(self, target, parent):
        """
        Add an interactive object into the xacro scene

        @param target (igraph.Vertex): target node to be added
        @param parent (igraph.Vertex): parent node (if exists) of the target node
        """
        obj_ns_prefix = self.get_link_name_(target)
        xacro_dir = self.get_interactive_object_dir_(target)

        self.main_xacro_.add_include(xacro_dir)

        kwargs = {
            "ns_prefix": obj_ns_prefix,
            "parent": self.get_link_name_(parent),
            "joint_type": "fixed",
            "xyz": "{} {} {}".format(*target["position"]),
            "rpy": "{} {} {}".format(*euler_from_quaternion(target["orientation"], "sxyz")),
            "scale": "{}".format(1)
        }
        self.main_xacro_.instantiate_macro(target["cad_id"], **kwargs)


    def add_concept_link_(self, target, parent):
        """
        Add a concept node into the xacro scene

        A concept node does not have an actual object associated with, which
        only contains a link frame in the xacro scene.

        @param target (igraph.Vertex): target node to be added
        @param parent (igraph.Vertex): parent node (if exists) of the target node
        """
        link_name = self.get_link_name_(target)
        
        # add concept node link into xacro file
        self.rigid_obj_xacro_.add_link(link_name, RIGID_OBJ_XACRO_MACRO_NAME)

        # add a joint (transform) between parent and child
        if parent is not None:
            self.rigid_obj_xacro_.add_joint(
                self.get_joint_name_(target, parent),
                RIGID_DEFAULT_JOINT_TYPE,
                link_name,
                self.get_link_name_(parent),
                target["position"],
                euler_from_quaternion(target["orientation"], "sxyz"),
                RIGID_OBJ_XACRO_MACRO_NAME
            )
        

    def create_main_xacro_(self, scene_name="simulation_scene"):
        """
        Create a xacro object for the main file

        @return (XacroROS) the xacro object of the main file
        """
        urdf_xacro = XacroROS(scene_name)
        
        return urdf_xacro


    def create_rigid_obj_xacro_(self):
        """
        Create a xacro object for rigid-body objects

        @return (XacroROS) the xacro object of the rigid objects
        """
        urdf_xacro = XacroROS(RIGID_OBJ_XACRO_MACRO_NAME)
        urdf_xacro.add_macro(RIGID_OBJ_XACRO_MACRO_NAME)

        return urdf_xacro

    
    def get_link_name_(self, node):
        """
        Generate link name for the given node

        Noted that link name should be unique in a xacro file

        @param node: (igraph.Vertex) node object
        @return (string) the link name
        """
        return "{}_{}_link".format(node["label"], node["id"])


    def get_joint_name_(self, target, parent):
        """
        Get the joint name of the joint

        @param target: (igraph.Vertex) child node
        @param parent: (igraph.Vertex) parent node
        @return (string) the joint name
        """
        assert(parent is not None)

        joint_name = "{}_{}_{}_{}_joint".format(
            parent["label"], parent["id"], target["label"], target["id"]
        )

        return joint_name


    def get_bg_mesh_dir_(self, node):
        """
        Get the directory of the background pointcloud of the node

        The directory format should followed the ROS convention, e.g,
            package://tooluse_viz/asset/mesh/hammer_t9.stl
        
        @param node (igraph.Vertex) node object that represents background
        @return (string) the directory of the background pointcloud
        """
        bg_mesh_dir = "{}/{}/{}/{}.ply".format(
            BACKGROUND_MESH_DIR_PREFIX, self.scene_name_, BACKGROUND_MESH_FOLDER, node["id"]
        )
        return bg_mesh_dir


    def get_mesh_dir_(self, node):
        """
        Get the directory of the 3D CAD model of the node

        The directory format should followed the ROS convention, e.g,
            package://tooluse_viz/asset/mesh/hammer_t9.stl
        
        @param node (igraph.Vertex) node object that represents rigid body
        @return (string) the directory of the CAD model
        """
        cad_id = node["cad_id"]

        mesh_dir = "{}/{}.obj".format(RIGID_MESH_DIR_PREFIX, cad_id)

        return mesh_dir

    
    def get_interactive_object_dir_(self, node):
        """
        Get the directory of the interactive object of the node

        The interactive object is stored in a xacro file
        
        @param node (igraph.Vertex) node object
        @return (string) the directory of the interactive object xacro file
        """
        cad_id = node["cad_id"]

        xacro_dir = "{}/{}/{}.xacro".format(XACRO_INC_INTERACTIVE_PREFIX, cad_id, cad_id)

        return xacro_dir
        

