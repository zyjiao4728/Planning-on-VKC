from lxml import etree as ET
from lxml.builder import ElementMaker



EM = ElementMaker(
    namespace="http://ros.org/wiki/xacro",
    nsmap={"xacro": "http://ros.org/wiki/xacro"}
)


class XacroROS(object):
    
    def __init__(self, root_name):
        self.root_ = ET.Element("robot", nsmap={"xacro": "http://ros.org/wiki/xacro"})
        
        # set root attributes
        self.root_.set("name", root_name)

        self.macros_ = {}
        

    def __str__(self):
        return ET.tostring(self.root_, pretty_print=True).decode()

    
    def save(self, filename):
        """
        Write current xacro into a file

        @param filename: (string) the output directory
        """
        tree = ET.ElementTree(self.root_)
        tree.write(filename, pretty_print=True, xml_declaration=True, encoding="utf-8")

        print("[INFO] xacro file is saved at: {}".format(filename))


    def add_macro(self, macro_name):
        """
        Define a new macro inside the xacro

        @param macro_name: (string) the name of the macro
        """
        macro = EM.macro()
        macro.set("name", macro_name)

        self.root_.append(macro)
        self.macros_[macro_name] = macro


    def add_include(self, inc_dir):
        """
        Add a include command into the xacro scene

        Include another xacro file before instantiate the macro defined in that file

        @param inc_dir: (string) the directory to be included 
        """
        inc = EM.include()
        inc.set("filename", inc_dir)

        self.root_.append(inc)


    def add_background(self, link_name, mesh_dir, xyz, rpy, scale=[1.0, 1.0, 1.0], ns=None):
        """
        Add a background object into xacro

        A background object only has visual element

        @param link_name: (string) the link name of the new added object
        @param mesh_dir: (string) directory of the object mesh model
        @param xyz: (list) a list of [x, y, z] that specifies the position of the origin tf
        @param rpy: (list) a list of [roll, pitch, yaw] that specifies the orientation of the origin tf
        @param scale: (list) a list of [x, y, z] that specify the scale along each axis
        @param ns: (string) under which namespace, the object to be added
        """
        parent = self.get_parent_element_(ns)
        
        link = ET.SubElement(parent, "link")
        link.set("name", link_name)

        visual = ET.SubElement(link, "visual")

        self.add_mesh_to_element_(visual, mesh_dir, xyz, rpy, scale)


    def add_object(self, link_name, mesh_dir, xyz, rpy, scale=[1.0, 1.0, 1.0], ns=None):
        """
        Add new object into xacro

        @param link_name: (string) the link name of the new added object
        @param mesh_dir: (string) directory of the object mesh model
        @param xyz: (list) a list of [x, y, z] that specifies the position of the origin tf
        @param rpy: (list) a list of [roll, pitch, yaw] that specifies the orientation of the origin tf
        @param scale: (list) a list of [x, y, z] that specify the scale along each axis
        @param ns: (string) under which namespace, the object to be added
        """
        parent = self.get_parent_element_(ns)
        
        link = ET.SubElement(parent, "link")
        link.set("name", link_name)

        visual = ET.SubElement(link, "visual")
        collision = ET.SubElement(link, "collision")

        self.add_mesh_to_element_(visual, mesh_dir, xyz, rpy, scale)
        self.add_mesh_to_element_(collision, mesh_dir, xyz, rpy, scale)


    def add_link(self, link_name, ns=None):
        """
        Add a single link into the xacro scene

        @param link_name: (string) the link name of the new added object
        @param ns: (string) under which namespace, the object to be added
        """
        parent = self.get_parent_element_(ns)

        link = ET.SubElement(parent, "link")
        link.set("name", link_name)


    def add_joint(self, joint_name, joint_type, child_link, parent_link, xyz, rpy, ns=None):
        """
        Add a joint (transform) between child link and parent link

        @param joint_name: (string) name of the joint
        @param child_link: (string): name of the child link
        @param parent_link: (string): name of the parent link
        @param xyz (list): position <x, y, z> of the transform
        @param rpy (list): euler angle <roll, pitch, yaw> of the transform
        """
        parent = self.get_parent_element_(ns)

        joint = ET.SubElement(parent, "joint")
        joint.set("name", joint_name)
        joint.set("type", joint_type)

        origin = ET.SubElement(joint, "origin")
        origin.set("xyz", "{} {} {}".format(*xyz))
        origin.set("rpy", "{} {} {}".format(*rpy))

        plink_element = ET.SubElement(joint, "parent")
        plink_element.set("link", parent_link)

        clink_element = ET.SubElement(joint, "child")
        clink_element.set("link", child_link)

    
    def instantiate_macro(self, macro_name, **kwargs):
        """
        Instantiate a pre-defined macro
        """
        ins_macro = ET.SubElement(self.root_, macro_name)

        for k, v in kwargs.items():
            ins_macro.set(k, v)


    def get_parent_element_(self, ns):
        """
        Add new object into xacro

        If ns is None, then the root element will be returned

        @param ns: (string) namespace
        """
        if ns is None:
            return self.root_
        else:
            assert(ns in self.macros_)
            return self.macros_[ns]


    def add_mesh_to_element_(self, element, mesh_dir, xyz, rpy, scale=[1.0, 1.0, 1.0]):
        """
        Add a mesh objects under the given element

        @param element: (ET.Element) the target element
        @param mesh_dir: (string): the directory of the mesh file
        @param xyz (list): position <x, y, z> of the transform
        @param rpy (list): euler angle <roll, pitch, yaw> of the transform
        @param scale (list): euler angle <roll, pitch, yaw> of the transform
        """
        origin = ET.SubElement(element, "origin")
        origin.set("xyz", "{} {} {}".format(*xyz))
        origin.set("rpy", "{} {} {}".format(*rpy))

        geom = ET.SubElement(element, "geometry")
        
        mesh = ET.SubElement(geom, "mesh")
        mesh.set("filename", mesh_dir)
        mesh.set("scale", "{} {} {}".format(*scale))


def test_xacro_ros():
    xacro = XacroROS("world")
    xacro.add_macro("create_world")
    xacro.add_object("create_world")
    print(xacro)


if __name__ == "__main__":
    test_xacro_ros()