MAIN_XACRO_FILENAME = "main.xacro"

RIGIT_OBJ_XACRO_FILENAME = "rigid_objects.xacro"
RIGID_OBJ_XACRO_MACRO_NAME = "create_rigid_objects"
RIGID_MESH_DIR_PREFIX = "package://scene_builder/assets/rigid"
RIGID_DEFAULT_XYZ = [0, 0, 0]
RIGID_DEFAULT_RPY = [0, 0, 0]
RIGID_DEFAULT_SCALE = [1, 1, 1]
RIGID_DEFAULT_JOINT_TYPE = "fixed"

XACRO_INC_PREFIX = "$(find scene_builder)/output"
XACRO_INC_INTERACTIVE_PREFIX = "$(find scene_builder)/assets/interactive"

BACKGROUND_MESH_DIR_PREFIX = "package://scene_builder/output"
BACKGROUND_MESH_FOLDER = "assets"

SCENE_PG_FILENAME = "contact_graph.json"
SCENE_SEGMENTS_DIR = "segments"

INTERACTIVE_CATEGORY = [
    "Cabinet",
    "Fridge",
    "Microwave",
    "Drawer",
    "Door",
    "Refrigerator"
]

BACKGROUND_CATEGORY = [
    "Room",
    "Ceiling",
    "Floor",
    "Wall"
]